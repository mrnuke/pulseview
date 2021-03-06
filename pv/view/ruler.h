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

#ifndef PULSEVIEW_PV_VIEW_RULER_H
#define PULSEVIEW_PV_VIEW_RULER_H

#include <QWidget>

namespace pv {
namespace view {

class TimeMarker;
class View;

class Ruler : public QWidget
{
	Q_OBJECT

private:
	static const int MinorTickSubdivision;
	static const int ScaleUnits[3];

	static const QString SIPrefixes[9];
	static const int FirstSIPrefixPower;

	static const int HoverArrowSize;

public:
	Ruler(View &parent);

	static QString format_time(double t, unsigned int prefix,
		unsigned precision = 0);

private:
	void paintEvent(QPaintEvent *event);

	void mouseMoveEvent(QMouseEvent *e);
	void mousePressEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *);

private:
	void draw_cursors(QPainter &p, unsigned int prefix);

	/**
	 * Draw a hover arrow under the cursor position.
	 */
	void draw_hover_mark(QPainter &p);

private slots:
	void hover_point_changed();

private:
	View &_view;

	TimeMarker *_grabbed_marker;
};

} // namespace view
} // namespace pv

#endif // PULSEVIEW_PV_VIEW_RULER_H
