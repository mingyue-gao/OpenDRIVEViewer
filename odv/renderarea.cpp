/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "renderarea.h"
#include "carla/opendrive/OpenDriveParser.h"
#include "carla/Memory.h"
#include "carla/geom/Vector2D.h"
#include "carla/road/element/Geometry.h"
#include "carla/road/element/RoadInfoGeometry.h"
#include "carla/road/element/RoadInfoVisitor.h"
#include "carla/road/MeshFactory.h"

#include <QtGui/QPainter>
#include <QtGui/QPainterPath>
#include <QtGui/QtGui>
#include <QEvent>
#include <QtWidgets/QtWidgets>
#include <qnamespace.h>

using namespace carla;
using namespace carla::road;

//! [0]
RenderArea::RenderArea(const odv::MapData& map_data, QWidget *parent)
    : QWidget(parent),
      // map_{opendrive::OpenDriveParser::LoadFile(map_path)},
      // mesh_(map_->GenerateMesh(10.)),
      //map_data_(odv::load_map(map_path))
      map_data_{map_data}
{
    QPalette pal;
    pal.setColor(QPalette::Background, Qt::gray);
    // setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);
    setPalette(pal);
}
//! [0]

//! [1]
QSize RenderArea::minimumSizeHint() const
{
    return QSize(100, 100);
}
//! [1]

//! [2]
QSize RenderArea::sizeHint() const
{
    return QSize(1920 * 2, 1680 * 2);
}
//! [2]

//! [3]

//! [7]
void RenderArea::wheelEvent(QWheelEvent * event)
{
  int numDegrees = event->delta() / 8;
  int numSteps = numDegrees / 15;
  scale_ += numSteps * 0.1;
  scale_ = std::max(scale_, 0.1);
  event->accept();
  update();
}
//

//! [8]
void RenderArea::paintEvent(QPaintEvent * /* event */)
{
    //static const QPoint points[4] = {
    //    QPoint(10, 80),
    //    QPoint(20, 10),
    //    QPoint(80, 30),
    //    QPoint(90, 70)
    //};
    //

    // QRect rect(10, 20, 80, 60);

    // QPainterPath path;
    // path.moveTo(20, 80);
    // path.lineTo(20, 30);
    // path.cubicTo(80, 0, 50, 50, 80, 80);

    // int startAngle = 20 * 16;
    // int arcLength = 120 * 16;
//! [8]

//! [9]
    QPainter painter(this);
    painter.setBrush(Qt::BrushStyle::SolidPattern);
    painter.setRenderHint(QPainter::Antialiasing, false);
//! [9]

//! [10]
    // auto& odr = map_->GetMap();
    // auto& roads = odr.GetRoads();

    painter.save();
    // painter.setBrush(Qt::SolidPattern);
    painter.translate(500, 500);
    // painter.scale(0.3, 0.3);
    painter.scale(scale_, scale_);

    //// Draw meshes
    // auto& vs = map_data_.mesh.GetVertices();
    // auto n = vs.size();

    // for (size_t i = 0; i < n - 2; ++i)
    // {
    //   const QPoint points3[3] = {QPoint(vs[i].x, vs[i].y),
    //                              QPoint(vs[i + 1].x, vs[i + 1].y),
    //                              QPoint(vs[i + 2].x, vs[i + 2].y)};
    //   painter.drawPolygon(points3, 3);
    // }
    //// Draw meshes DONE

    auto& lbs = map_data_.lane_boundaries;
    for (auto& l : map_data_.lanes)
    {
      auto n = l.meshes.size();
      for (std::size_t i = 0; i < n - 2; ++i)
      {
        auto& mshs = l.meshes;
        auto& p1 = lbs[mshs[i].first][mshs[i].second];
        auto& p2 = lbs[mshs[i + 1].first][mshs[i + 1].second];
        auto& p3 = lbs[mshs[i + 2].first][mshs[i + 2].second];

        const QPointF points3[3] = {p1, p2, p3};
        painter.drawPolygon(points3, 3);
      }
    }

    QPen the_pen(Qt::white);
    the_pen.setWidthF(0.5);
    the_pen.setStyle(Qt::PenStyle::DashLine);
    painter.setPen(the_pen);
    // for (auto& lb : lbs)
    // {
    //   painter.drawPolyline(lb.data(), lb.size());
    // }
    for (auto& l : map_data_.lanes)
    {
      auto& llb = map_data_.lane_boundaries[l.right_boundary];
      auto& rlb = map_data_.lane_boundaries[l.left_boundary];
      painter.drawPolyline(llb.data(), llb.size());
      painter.drawPolyline(rlb.data(), rlb.size());
    }

    painter.restore();
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.setPen(palette().dark().color());
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(QRect(0, 0, width() - 1, height() - 1));
}
//! [13]
