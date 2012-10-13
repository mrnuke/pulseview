/*
 * This file is part of the PulseView project.
 *
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include "sigsession.h"

#include "logicdata.h"
#include "logicdatasnapshot.h"
#include "view/logicsignal.h"

#include <QDebug>

#include <assert.h>

using namespace boost;
using namespace std;

namespace pv {

int SigSession::HwCaps[] = {
	SR_HWCAP_SAMPLERATE,
	0
};

// TODO: This should not be necessary
SigSession* SigSession::_session = NULL;

SigSession::SigSession() :
	_capture_state(Stopped)
{
	// TODO: This should not be necessary
	_session = this;
}

SigSession::~SigSession()
{
	stop_capture();

	if (_sampling_thread.get())
		_sampling_thread->join();
	_sampling_thread.reset();

	// TODO: This should not be necessary
	_session = NULL;
}

void SigSession::load_file(const std::string &name)
{
	if (sr_session_load(name.c_str()) == SR_OK) {
		/* sigrok session file */
		sr_session_datafeed_callback_add(data_feed_in_proc);
		sr_session_start();
		sr_session_run();
		sr_session_stop();
	}
}

SigSession::capture_state SigSession::get_capture_state() const
{
	lock_guard<mutex> lock(_state_mutex);
	return _capture_state;
}

void SigSession::save_file(const string &name)
{
	if(!_logic_data) {
		qDebug() << "No _logic_data to save.";
		return;
	}

	deque< shared_ptr<LogicDataSnapshot> > snapshots =
		_logic_data->get_snapshots();
	if(snapshots.empty()) {
		qDebug() << "No snapshots to save.";
		return;
	}

	const shared_ptr<LogicDataSnapshot> &snapshot = snapshots.front();
	if(!snapshot) {
		qDebug() << "snapshot is invalid.";
		return;
	}

	const sr_dev_inst *const sdi = get_sr_dev_inst();

	SaveCallbackState state = {
		this,
		snapshot->get_unit_size() * snapshot->get_sample_count(),
		(uint8_t*)snapshot->get_data()
	};

	if (sr_session_save(name.c_str(), sdi, snapshot->get_unit_size(),
		save_data_callback, &state) != SR_OK)
		qDebug() << "Failed to store session.";

	release_sr_dev_inst(sdi);
}

void SigSession::start_capture(struct sr_dev_inst *sdi,
	uint64_t record_length, uint64_t sample_rate)
{
	stop_capture();


	_sampling_thread.reset(new boost::thread(
		&SigSession::sample_thread_proc, this, sdi,
		record_length, sample_rate));
}

void SigSession::stop_capture()
{
	if (get_capture_state() == Stopped)
		return;

	sr_session_stop();

	// Check that sampling stopped
	if (_sampling_thread.get())
		_sampling_thread->join();
	_sampling_thread.reset();
}

vector< shared_ptr<view::Signal> > SigSession::get_signals()
{
	lock_guard<mutex> lock(_signals_mutex);
	return _signals;
}

boost::shared_ptr<LogicData> SigSession::get_data()
{
	return _logic_data;
}

void SigSession::set_capture_state(capture_state state)
{
	lock_guard<mutex> lock(_state_mutex);
	_capture_state = state;
	capture_state_changed(state);
}

void SigSession::sample_thread_proc(struct sr_dev_inst *sdi,
	uint64_t record_length, uint64_t sample_rate)
{
	sr_session_new();
	sr_session_datafeed_callback_add(data_feed_in_proc);

	if (sr_session_dev_add(sdi) != SR_OK) {
		qDebug() << "Failed to use device.";
		sr_session_destroy();
		return;
	}

	if (sr_dev_config_set(sdi, SR_HWCAP_LIMIT_SAMPLES,
		&record_length) != SR_OK) {
		qDebug() << "Failed to configure time-based sample limit.";
		sr_session_destroy();
		return;
	}

	if (sr_dev_config_set(sdi, SR_HWCAP_SAMPLERATE,
		&sample_rate) != SR_OK) {
		qDebug() << "Failed to configure samplerate.";
		sr_session_destroy();
		return;
	}

	if (sr_session_start() != SR_OK) {
		qDebug() << "Failed to start session.";
		return;
	}

	set_capture_state(Running);

	sr_session_run();
	sr_session_destroy();

	set_capture_state(Stopped);
}

void SigSession::data_feed_in(const struct sr_dev_inst *sdi,
	struct sr_datafeed_packet *packet)
{
	using view::LogicSignal;

	assert(sdi);
	assert(packet);

	switch (packet->type) {
	case SR_DF_HEADER:
	{
		lock_guard<mutex> lock(_signals_mutex);
		_signals.clear();
		break;
	}

	case SR_DF_META_LOGIC:
	{
		assert(packet->payload);
		const sr_datafeed_meta_logic &meta_logic =
			*(sr_datafeed_meta_logic*)packet->payload;

	{
		lock_guard<mutex> lock(_data_mutex);

		// Create an empty LogiData for coming data snapshots
		_logic_data.reset(new LogicData(meta_logic));
		assert(_logic_data);
		if (!_logic_data)
			break;
	}

	{
		lock_guard<mutex> lock(_signals_mutex);

		// Add the signals
		for (int i = 0; i < meta_logic.num_probes; i++)
		{
			const sr_probe *const probe =
				(const sr_probe*)g_slist_nth_data(
					sdi->probes, i);
			if (probe->enabled)
			{
				shared_ptr<LogicSignal> signal(
					new LogicSignal(probe->name,
						_logic_data,
						probe->index));
				_signals.push_back(signal);
			}
		}

		signals_changed();
		break;
	}
	}

	case SR_DF_LOGIC:
	{
		lock_guard<mutex> lock(_data_mutex);
		assert(packet->payload);
		if (!_cur_logic_snapshot)
		{
			// Create a new data snapshot
			_cur_logic_snapshot = shared_ptr<LogicDataSnapshot>(
				new LogicDataSnapshot(
				*(sr_datafeed_logic*)packet->payload));
			_logic_data->push_snapshot(_cur_logic_snapshot);
		}
		else
		{
			// Append to the existing data snapshot
			_cur_logic_snapshot->append_payload(
				*(sr_datafeed_logic*)packet->payload);
		}

		data_updated();
		break;
	}

	case SR_DF_END:
	{
		{
			lock_guard<mutex> lock(_data_mutex);
			_cur_logic_snapshot.reset();
		}
		data_updated();
		break;
	}
	}
}

void SigSession::data_feed_in_proc(const struct sr_dev_inst *sdi,
	struct sr_datafeed_packet *packet)
{
	assert(_session);
	_session->data_feed_in(sdi, packet);
}

int SigSession::info_get(int info_id, const void **data,
	const struct sr_dev_inst *sdi)
{
	// Respond to instance-less info requests
	switch(info_id) {
	case SR_DI_HWCAPS:
		*data = HwCaps;
		return SR_OK;
	}

	if(!sdi)
		return SR_ERR_ARG;

	// Respond to instanced info requests
	DevInstPriv *const priv = (DevInstPriv*)sdi->priv;
	assert(priv);

	switch(info_id) {
	case SR_DI_CUR_SAMPLERATE:
		*(uint64_t**)data = &priv->sample_rate;
		return SR_OK;
	}

	return SR_ERR_ARG;
}

struct sr_dev_inst* SigSession::get_sr_dev_inst()
{
	assert(_logic_data);

	struct sr_dev_inst *sdi = new sr_dev_inst;
	memset(sdi, 0, sizeof(sr_dev_inst));

	sr_dev_driver *driver = new sr_dev_driver;
	memset(driver, 0, sizeof(sr_dev_driver));
	driver->info_get = info_get;

	DevInstPriv *const priv = new DevInstPriv;
	priv->session = this;
	priv->sample_rate = _logic_data->get_samplerate();

	for(int i = _signals.size() - 1; i >= 0; i--) {
		const shared_ptr<view::Signal> s = _signals[i];
		assert(s);

		sr_probe *const probe = new struct sr_probe;

		probe->index = i;
		probe->type = SR_PROBE_LOGIC;
		probe->enabled = TRUE;
		probe->trigger = NULL;

		const QString name = s->get_name();
		probe->name = strdup(name.toUtf8().data());

		sdi->probes = g_slist_prepend(sdi->probes, probe);
	}

	sdi->priv = priv;
	sdi->driver = driver;

	return sdi;
}

void SigSession::release_sr_dev_inst(const struct sr_dev_inst *const sdi)
{
	assert(sdi);
	assert(sdi->driver);
	assert(sdi->probes);

	for(GSList *p = sdi->probes; p; p = p->next) {
		sr_probe *const probe = (sr_probe*)p->data;

		assert(probe->name);
		free(probe->name);

		delete probe;
	}

	g_slist_free(sdi->probes);

	delete sdi->driver;
	delete (DevInstPriv*)sdi->priv;
	delete sdi;
}

ssize_t SigSession::save_data_callback(uint16_t type,
	void *data, size_t len, void *cb_data)
{
	assert(cb_data);
	SaveCallbackState *const state = (SaveCallbackState*)cb_data;

	switch(type) {
	case SR_DS_BEGIN:
		return SR_OK;

	case SR_DS_READ:
	{
		const ssize_t read_length = min(len, state->datasize);
		memcpy(data, state->data, read_length);
		state->datasize -= read_length;
		state->data += len;
		return read_length;
	}

	case SR_DS_END:
		return SR_OK;

	case SR_DS_ERROR:
		return SR_OK;
	};

	return SR_ERR;
}

} // namespace pv
