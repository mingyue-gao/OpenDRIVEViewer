#include "carla/opendrive/OpenDriveParser.h"

#include <CImg/CImg.h>

#include <vector>
#include <chrono>
#include <iostream>

using namespace cimg_library;
using namespace carla;

template <typename T, typename tp, typename tf, typename tc, typename to>
const CImg<T>& _display_object3d(
    int argc,
    const char** argv,
    CImg<T>& img,
    CImgDisplay& disp, const char* const title, CImg<tp> vertices,
    CImgList<tf> primitives, CImgList<tc> colors,
    const to& opacities, const bool centering, const int render_static,
    const int render_motion, const bool is_double_sided, const float focale,
    const float light_x, const float light_y, const float light_z,
    const float specular_lightness, const float specular_shininess,
    const bool display_axes, float* const pose_matrix,
    const bool exit_on_anykey)
{
  typedef typename cimg::superset<tp, float>::type tpfloat;
  typedef typename CImg<T>::boolT boolT;
  typedef typename CImg<T>::charT charT;
  typedef typename CImg<T>::ucharT ucharT;
  typedef typename CImg<T>::shortT shortT;
  typedef typename CImg<T>::ushortT ushortT;
  typedef typename CImg<T>::intT intT;
  typedef typename CImg<T>::uintT uintT;
  typedef typename CImg<T>::longT longT;
  typedef typename CImg<T>::ulongT ulongT;
  typedef typename CImg<T>::floatT floatT;
  typedef typename CImg<T>::doubleT doubleT;

  const auto _width = img.width();
  const auto _height = img.height();
  const auto _depth = img.depth();
  const auto _pixel_type = img.pixel_type();
  const auto _spectrum = img.spectrum();

  // Check input arguments
  if (img.is_empty())
  {
    if (disp)
      return CImg<T>(disp.width(), disp.height(), 1,
                     (colors && colors[0].size() == 1) ? 1 : 3, 0)
          ._display_object3d(disp, title, vertices, primitives, colors,
                             opacities, centering, render_static, render_motion,
                             is_double_sided, focale, light_x, light_y, light_z,
                             specular_lightness, specular_shininess,
                             display_axes, pose_matrix, exit_on_anykey);
    else
      return CImg<T>(1, 2, 1, 1, 64, 128)
          .resize(cimg_fitscreen(CImgDisplay::screen_width() / 2,
                                 CImgDisplay::screen_height() / 2, 1),
                  1, (colors && colors[0].size() == 1) ? 1 : 3, 3)
          ._display_object3d(disp, title, vertices, primitives, colors,
                             opacities, centering, render_static, render_motion,
                             is_double_sided, focale, light_x, light_y, light_z,
                             specular_lightness, specular_shininess,
                             display_axes, pose_matrix, exit_on_anykey);
  }
  else
  {
    if (disp) disp.resize(img, false);
  }
  // CImg<charT> error_message(1024);
  // if (!vertices.is_object3d(primitives, colors, opacities, true, error_message))
  //   throw CImgArgumentException(
  //       _cimg_instance
  //       "display_object3d(): Invalid specified 3D object (%u,%u) (%s).",
  //       cimg_instance, vertices._width, primitives._width,
  //       error_message.data());
  // if (vertices._width && !primitives)
  // {
  //   CImgList<tf> nprimitives(vertices._width, 1, 1, 1, 1);
  //   cimglist_for(nprimitives, l) nprimitives(l, 0) = (tf)l;
  //   return _display_object3d(disp, title, vertices, nprimitives, colors,
  //                            opacities, centering, render_static, render_motion,
  //                            is_double_sided, focale, light_x, light_y, light_z,
  //                            specular_lightness, specular_shininess,
  //                            display_axes, pose_matrix, exit_on_anykey);
  // }
  if (!disp)
  {
    disp.assign(cimg_fitscreen(_width, _height, _depth), title ? title : 0, 3);
    if (!title)
      disp.set_title("CImg<%s> (%u vertices, %u primitives)", _pixel_type,
                     vertices._width, primitives._width);
  }
  else if (title)
    disp.set_title("%s", title);

  CImg<floatT> pose, bbox_vertices,
      rotated_bbox_vertices, axes_vertices, rotated_axes_vertices,
      bbox_opacities, axes_opacities;
  CImgList<uintT> bbox_primitives, axes_primitives;
  CImgList<tf> reverse_primitives;
  CImgList<T> bbox_colors, bbox_colors2, axes_colors;
  unsigned int ns_width = 0, ns_height = 0;
  int _is_double_sided = (int)is_double_sided;
  bool ndisplay_axes = display_axes;
  const CImg<T> background_color(1, 1, 1, _spectrum, 0),
      foreground_color(1, 1, 1, _spectrum,
                       (T)std::min((int)cimg::type<T>::max(), 255));
  float Xoff = 0, Yoff = 0, Zoff = 0, sprite_scale = 1, xm = 0;
  float ym = 0, zm = 0;

  bbox_primitives.assign(6, 1, 4, 1, 1, 0, 3, 2, 1, 4, 5, 6, 7, 1, 2, 6, 5, 0,
                         4, 7, 3, 0, 1, 5, 4, 2, 3, 7, 6);
  bbox_colors.assign(6, _spectrum, 1, 1, 1, background_color[0]);
  bbox_colors2.assign(6, _spectrum, 1, 1, 1, foreground_color[0]);
  bbox_opacities.assign(bbox_colors._width, 1, 1, 1, 0.3f);

  rotated_axes_vertices =
      axes_vertices.assign(7, 3, 1, 1, 0, 20, 0, 0, 22, -6, -6, 0, 0, 20, 0, -6,
                           22, -6, 0, 0, 0, 20, 0, 0, 22);
  axes_opacities.assign(3, 1, 1, 1, 1);
  axes_colors.assign(3, _spectrum, 1, 1, 1, foreground_color[0]);
  axes_primitives.assign(3, 1, 2, 1, 1, 0, 1, 0, 2, 0, 3);



  // Begin user interaction loop
  CImg<T> visu0(img, false), visu;
  CImg<tpfloat> zbuffer(visu0.width(), visu0.height(), 1, 1, 0);
  bool init_pose = true, clicked = false, redraw = true;
  unsigned int key = 0, font_size = 32;
  int x0 = 0, y0 = 0, x1 = 0, y1 = 0, nrender_static = render_static,
      nrender_motion = render_motion;
  disp.show().flush();

  const CImg<tp> origin_vertices = vertices;
  const CImgList<tf> origin_primitives = primitives;
  const CImgList<tc> origin_colors = colors;

  std::vector<CImgList<unsigned int>> vehicles_prims;
  std::vector<CImg<float>> vehicles_pts;
  while (!disp.is_closed() && !key)
  {
    vertices = origin_vertices;
    primitives = origin_primitives;

    // CImgList<unsigned int> cube_prims;
    // const CImg<float> cube_pts =
    //     CImg<>::box3d(cube_prims, 50, 50, 50);
    // vertices.append_object3d(primitives, cube_pts, cube_prims);

    vehicles_prims.clear();
    vehicles_pts.clear();
    // const auto vehicles = t_model.update();
    // for (const auto& v : vehicles)
    // {
    //   if (v.id == 0)
    //     continue;

    //   CImgList<unsigned int> cube_prims;
    //   const CImg<float> cube_pts =
    //       CImg<>::box3d(cube_prims, 5, 2.5, 2)
    //           .shift_object3d()
    //           .shift_object3d(v.pos.x, v.pos.y, v.pos.z);
    //   // std::cout << "vid=" << v.id << " x=" << v.pos.x << " y=" << v.pos.y << std::endl;
    //   vehicles_prims.push_back(std::move(cube_prims));
    //   vehicles_pts.push_back(std::move(cube_pts));
    // }

    int n = vehicles_prims.size();
    for (int i = 0; i < n; ++i)
    {
      vertices.append_object3d(primitives, vehicles_pts[i], vehicles_prims[i]);
    }

    redraw = true;

  auto frame_start = std::chrono::high_resolution_clock::now();

  // Init 3D objects and compute object statistics
  CImg<floatT> rotated_vertices(vertices._width, 3);
  float xM = vertices ? vertices.get_shared_row(0).max_min(xm) : 0,
        yM = vertices ? vertices.get_shared_row(1).max_min(ym) : 0,
        zM = vertices ? vertices.get_shared_row(2).max_min(zm) : 0;
  const float delta = cimg::max(xM - xm, yM - ym, zM - zm);

  rotated_bbox_vertices = bbox_vertices.assign(
      8, 3, 1, 1, xm, xM, xM, xm, xm, xM, xM, xm, ym, ym, yM, yM, ym, ym, yM,
      yM, zm, zm, zm, zm, zM, zM, zM, zM);


    // Init object pose
    if (init_pose)
    {
      const float ratio = delta > 0
                              ? (2.f * std::min(disp.width(), disp.height()) /
                                 (3.f * delta))
                              : 1,
                  dx = (xM + xm) / 2, dy = (yM + ym) / 2, dz = (zM + zm) / 2;
      if (centering)
        CImg<floatT>(4, 3, 1, 1, ratio, 0., 0., -ratio * dx, 0., ratio, 0.,
                     -ratio * dy, 0., 0., ratio, -ratio * dz)
            .move_to(pose);
      else
        CImg<floatT>(4, 3, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0)
            .move_to(pose);
      if (pose_matrix)
      {
        CImg<floatT> pose0(pose_matrix, 4, 3, 1, 1, false);
        pose0.resize(4, 4, 1, 1, 0);
        pose.resize(4, 4, 1, 1, 0);
        pose0(3, 3) = pose(3, 3) = 1;
        (pose0 * pose).get_crop(0, 0, 3, 2).move_to(pose);
        Xoff = pose_matrix[12];
        Yoff = pose_matrix[13];
        Zoff = pose_matrix[14];
        sprite_scale = pose_matrix[15];
      }
      else
      {
        Xoff = Yoff = Zoff = 0;
        sprite_scale = 1;
      }
      init_pose = false;
      redraw = true;
    }

    // Rotate and draw 3D object
    if (redraw)
    {
      const float r00 = pose(0, 0), r10 = pose(1, 0), r20 = pose(2, 0),
                  r30 = pose(3, 0), r01 = pose(0, 1), r11 = pose(1, 1),
                  r21 = pose(2, 1), r31 = pose(3, 1), r02 = pose(0, 2),
                  r12 = pose(1, 2), r22 = pose(2, 2), r32 = pose(3, 2);
      if ((clicked && nrender_motion >= 0) || (!clicked && nrender_static >= 0))
        cimg_forX(vertices, l)
        {
          const float x = (float)vertices(l, 0), y = (float)vertices(l, 1),
                      z = (float)vertices(l, 2);
          rotated_vertices(l, 0) = r00 * x + r10 * y + r20 * z + r30;
          rotated_vertices(l, 1) = r01 * x + r11 * y + r21 * z + r31;
          rotated_vertices(l, 2) = r02 * x + r12 * y + r22 * z + r32;
        }
      else
        cimg_forX(bbox_vertices, l)
        {
          const float x = bbox_vertices(l, 0), y = bbox_vertices(l, 1),
                      z = bbox_vertices(l, 2);
          rotated_bbox_vertices(l, 0) = r00 * x + r10 * y + r20 * z + r30;
          rotated_bbox_vertices(l, 1) = r01 * x + r11 * y + r21 * z + r31;
          rotated_bbox_vertices(l, 2) = r02 * x + r12 * y + r22 * z + r32;
        }

      // Draw objects
      const bool render_with_zbuffer = !clicked && nrender_static > 0;
      visu = visu0;
      if ((clicked && nrender_motion < 0) || (!clicked && nrender_static < 0))
        visu.draw_object3d(Xoff + visu._width / 2.f, Yoff + visu._height / 2.f,
                           Zoff, rotated_bbox_vertices, bbox_primitives,
                           bbox_colors, bbox_opacities, 2, false, focale)
            .draw_object3d(Xoff + visu._width / 2.f, Yoff + visu._height / 2.f,
                           Zoff, rotated_bbox_vertices, bbox_primitives,
                           bbox_colors2, 1, false, focale);
      else
        visu._draw_object3d(
            (void*)0,
            render_with_zbuffer ? zbuffer.fill(0) : CImg<tpfloat>::empty(),
            Xoff + visu._width / 2.f, Yoff + visu._height / 2.f, Zoff,
            rotated_vertices,
            reverse_primitives ? reverse_primitives : primitives, colors,
            opacities, clicked ? nrender_motion : nrender_static,
            _is_double_sided == 1, focale, _width / 2.f + light_x,
            _height / 2.f + light_y, light_z + Zoff, specular_lightness,
            specular_shininess, 1, sprite_scale);
      // Draw axes
      if (ndisplay_axes)
      {
        const float n = 1e-8f + cimg::hypot(r00, r01, r02), _r00 = r00 / n,
                    _r10 = r10 / n, _r20 = r20 / n, _r01 = r01 / n,
                    _r11 = r11 / n, _r21 = r21 / n, _r02 = r01 / n,
                    _r12 = r12 / n, _r22 = r22 / n, Xaxes = 25,
                    Yaxes = visu._height - 38.f;
        cimg_forX(axes_vertices, l)
        {
          const float x = axes_vertices(l, 0), y = axes_vertices(l, 1),
                      z = axes_vertices(l, 2);
          rotated_axes_vertices(l, 0) = _r00 * x + _r10 * y + _r20 * z;
          rotated_axes_vertices(l, 1) = _r01 * x + _r11 * y + _r21 * z;
          rotated_axes_vertices(l, 2) = _r02 * x + _r12 * y + _r22 * z;
        }
        axes_opacities(0, 0) = (rotated_axes_vertices(1, 2) > 0) ? 0.5f : 1.f;
        axes_opacities(1, 0) = (rotated_axes_vertices(2, 2) > 0) ? 0.5f : 1.f;
        axes_opacities(2, 0) = (rotated_axes_vertices(3, 2) > 0) ? 0.5f : 1.f;
        visu.draw_object3d(Xaxes, Yaxes, 0, rotated_axes_vertices,
                           axes_primitives, axes_colors, axes_opacities, 1,
                           false, focale)
            .draw_text((int)(Xaxes + rotated_axes_vertices(4, 0)),
                       (int)(Yaxes + rotated_axes_vertices(4, 1)), "X",
                       axes_colors[0]._data, 0, axes_opacities(0, 0), 13)
            .draw_text((int)(Xaxes + rotated_axes_vertices(5, 0)),
                       (int)(Yaxes + rotated_axes_vertices(5, 1)), "Y",
                       axes_colors[1]._data, 0, axes_opacities(1, 0), 13)
            .draw_text((int)(Xaxes + rotated_axes_vertices(6, 0)),
                       (int)(Yaxes + rotated_axes_vertices(6, 1)), "Z",
                       axes_colors[2]._data, 0, axes_opacities(2, 0), 13);
      }
      visu.display(disp);
      // if (!clicked || nrender_motion == nrender_static) redraw = false;
    }

    // Handle user interaction
    if (!redraw) disp.wait();
    if ((disp.button() || disp.wheel()) && disp.mouse_x() >= 0 &&
        disp.mouse_y() >= 0)
    {
      redraw = true;
      if (!clicked)
      {
        x0 = x1 = disp.mouse_x();
        y0 = y1 = disp.mouse_y();
        if (!disp.wheel()) clicked = true;
      }
      else
      {
        x1 = disp.mouse_x();
        y1 = disp.mouse_y();
      }
      const bool is_keyCTRL = disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT();
      if (disp.button() & 1 && !is_keyCTRL)
      {
        const float R = 0.45f * std::min(disp.width(), disp.height()),
                    R2 = R * R, u0 = (float)(x0 - disp.width() / 2),
                    v0 = (float)(y0 - disp.height() / 2),
                    u1 = (float)(x1 - disp.width() / 2),
                    v1 = (float)(y1 - disp.height() / 2),
                    n0 = cimg::hypot(u0, v0), n1 = cimg::hypot(u1, v1),
                    nu0 = n0 > R ? (u0 * R / n0) : u0,
                    nv0 = n0 > R ? (v0 * R / n0) : v0,
                    nw0 = (float)std::sqrt(
                        std::max(0.f, R2 - nu0 * nu0 - nv0 * nv0)),
                    nu1 = n1 > R ? (u1 * R / n1) : u1,
                    nv1 = n1 > R ? (v1 * R / n1) : v1,
                    nw1 = (float)std::sqrt(
                        std::max(0.f, R2 - nu1 * nu1 - nv1 * nv1)),
                    u = nv0 * nw1 - nw0 * nv1, v = nw0 * nu1 - nu0 * nw1,
                    w = nv0 * nu1 - nu0 * nv1, n = cimg::hypot(u, v, w),
                    alpha = (float)std::asin(n / R2) * 180 / cimg::PI;
        (CImg<floatT>::rotation_matrix(u, v, w, -alpha) * pose).move_to(pose);
        x0 = x1;
        y0 = y1;
      }
      if (disp.button() & 2 && !is_keyCTRL)
      {
        if (focale > 0)
          Zoff -= (y0 - y1) * focale / 400;
        else
        {
          const float s = std::exp((y0 - y1) / 400.f);
          pose *= s;
          sprite_scale *= s;
        }
        x0 = x1;
        y0 = y1;
      }
      if (disp.wheel())
      {
        if (focale > 0)
          Zoff -= disp.wheel() * focale / 20;
        else
        {
          const float s = std::exp(disp.wheel() / 20.f);
          pose *= s;
          sprite_scale *= s;
        }
        disp.set_wheel();
      }
      if (disp.button() & 4 || (disp.button() & 1 && is_keyCTRL))
      {
        Xoff += (x1 - x0);
        Yoff += (y1 - y0);
        x0 = x1;
        y0 = y1;
      }
      if ((disp.button() & 1) && (disp.button() & 2) && !is_keyCTRL)
      {
        init_pose = true;
        disp.set_button();
        x0 = x1;
        y0 = y1;
        pose = CImg<floatT>(4, 3, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
      }
    }
    else if (clicked)
    {
      x0 = x1;
      y0 = y1;
      clicked = false;
      redraw = true;
    }

    CImg<charT> filename(32);
    switch (key = disp.key())
    {
#if cimg_OS != 2
      case cimg::keyCTRLRIGHT:
#endif
      case 0:
      case cimg::keyCTRLLEFT:
        key = 0;
        break;
      case cimg::keyD:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {
          disp.set_fullscreen(false)
              .resize(CImgDisplay::_fitscreen(3 * disp.width() / 2,
                                              3 * disp.height() / 2, 1, 128,
                                              -100, false),
                      CImgDisplay::_fitscreen(3 * disp.width() / 2,
                                              3 * disp.height() / 2, 1, 128,
                                              -100, true),
                      false)
              ._is_resized = true;
          disp.set_key(key, false);
          key = 0;
        }
        break;
      case cimg::keyC:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {
          disp.set_fullscreen(false)
              .resize(cimg_fitscreen(2 * disp.width() / 3,
                                     2 * disp.height() / 3, 1),
                      false)
              ._is_resized = true;
          disp.set_key(key, false);
          key = 0;
        }
        break;
      case cimg::keyR:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {
          disp.set_fullscreen(false)
              .resize(cimg_fitscreen(_width, _height, _depth), false)
              ._is_resized = true;
          disp.set_key(key, false);
          key = 0;
        }
        break;
      case cimg::keyF:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {
          if (!ns_width || !ns_height ||
              ns_width > (unsigned int)disp.screen_width() ||
              ns_height > (unsigned int)disp.screen_height())
          {
            ns_width = disp.screen_width() * 3U / 4;
            ns_height = disp.screen_height() * 3U / 4;
          }
          if (disp.is_fullscreen())
            disp.resize(ns_width, ns_height, false);
          else
          {
            ns_width = disp._width;
            ns_height = disp._height;
            disp.resize(disp.screen_width(), disp.screen_height(), false);
          }
          disp.toggle_fullscreen()._is_resized = true;
          disp.set_key(key, false);
          key = 0;
        }
        break;
      case cimg::keyT:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {
          // Switch single/double-sided primitives.
          if (--_is_double_sided == -2) _is_double_sided = 1;
          if (_is_double_sided >= 0)
            reverse_primitives.assign();
          else
            primitives.get_reverse_object3d().move_to(reverse_primitives);
          disp.set_key(key, false);
          key = 0;
          redraw = true;
        }
        break;
      case cimg::keyZ:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {  // Enable/disable Z-buffer
          if (zbuffer)
            zbuffer.assign();
          else
            zbuffer.assign(visu0.width(), visu0.height(), 1, 1, 0);
          disp.set_key(key, false);
          key = 0;
          redraw = true;
        }
        break;
      case cimg::keyX:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {  // Show/hide 3D axes
          ndisplay_axes = !ndisplay_axes;
          disp.set_key(key, false);
          key = 0;
          redraw = true;
        }
        break;
      case cimg::keyF1:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {  // Set rendering mode to points
          nrender_motion =
              (nrender_static == 0 && nrender_motion != 0) ? 0 : -1;
          nrender_static = 0;
          disp.set_key(key, false);
          key = 0;
          redraw = true;
        }
        break;
      case cimg::keyF2:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {  // Set rendering mode to lines
          nrender_motion =
              (nrender_static == 1 && nrender_motion != 1) ? 1 : -1;
          nrender_static = 1;
          disp.set_key(key, false);
          key = 0;
          redraw = true;
        }
        break;
      case cimg::keyF3:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {  // Set rendering mode to flat
          nrender_motion =
              (nrender_static == 2 && nrender_motion != 2) ? 2 : -1;
          nrender_static = 2;
          disp.set_key(key, false);
          key = 0;
          redraw = true;
        }
        break;
      case cimg::keyF4:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {  // Set rendering mode to flat-shaded
          nrender_motion =
              (nrender_static == 3 && nrender_motion != 3) ? 3 : -1;
          nrender_static = 3;
          disp.set_key(key, false);
          key = 0;
          redraw = true;
        }
        break;
      case cimg::keyF5:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {
          // Set rendering mode to gouraud-shaded.
          nrender_motion =
              (nrender_static == 4 && nrender_motion != 4) ? 4 : -1;
          nrender_static = 4;
          disp.set_key(key, false);
          key = 0;
          redraw = true;
        }
        break;
      case cimg::keyF6:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {  // Set rendering mode to phong-shaded
          nrender_motion =
              (nrender_static == 5 && nrender_motion != 5) ? 5 : -1;
          nrender_static = 5;
          disp.set_key(key, false);
          key = 0;
          redraw = true;
        }
        break;
      case cimg::keyS:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {  // Save snapshot
          static unsigned int snap_number = 0;
          std::FILE* file;
          do
          {
            cimg_snprintf(filename, filename._width, cimg_appname "_%.4u.bmp",
                          snap_number++);
            if ((file = cimg::std_fopen(filename, "r")) != 0)
              cimg::fclose(file);
          } while (file);
          (+visu)
              .__draw_text(" Saving snapshot... ", font_size, 0)
              .display(disp);
          visu.save(filename);
          (+visu)
              .__draw_text(" Snapshot '%s' saved. ", font_size, 0,
                           filename._data)
              .display(disp);
          disp.set_key(key, false);
          key = 0;
        }
        break;
      case cimg::keyG:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {  // Save object as a .off file
          static unsigned int snap_number = 0;
          std::FILE* file;
          do
          {
            cimg_snprintf(filename, filename._width, cimg_appname "_%.4u.off",
                          snap_number++);
            if ((file = cimg::std_fopen(filename, "r")) != 0)
              cimg::fclose(file);
          } while (file);
          (+visu).__draw_text(" Saving object... ", font_size, 0).display(disp);
          vertices.save_off(
              reverse_primitives ? reverse_primitives : primitives, colors,
              filename);
          (+visu)
              .__draw_text(" Object '%s' saved. ", font_size, 0, filename._data)
              .display(disp);
          disp.set_key(key, false);
          key = 0;
        }
        break;
      case cimg::keyO:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {  // Save object as a .cimg file
          static unsigned int snap_number = 0;
          std::FILE* file;
          do
          {
#ifdef cimg_use_zlib
            cimg_snprintf(filename, filename._width, cimg_appname "_%.4u.cimgz",
                          snap_number++);
#else
            cimg_snprintf(filename, filename._width, cimg_appname "_%.4u.cimg",
                          snap_number++);
#endif
            if ((file = cimg::std_fopen(filename, "r")) != 0)
              cimg::fclose(file);
          } while (file);
          (+visu).__draw_text(" Saving object... ", font_size, 0).display(disp);
          vertices
              .get_object3dtoCImg3d(
                  reverse_primitives ? reverse_primitives : primitives, colors,
                  opacities)
              .save(filename);
          (+visu)
              .__draw_text(" Object '%s' saved. ", font_size, 0, filename._data)
              .display(disp);
          disp.set_key(key, false);
          key = 0;
        }
        break;

#ifdef cimg_use_board
      case cimg::keyP:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {  // Save object as a .EPS file
          static unsigned int snap_number = 0;
          std::FILE* file;
          do
          {
            cimg_snprintf(filename, filename._width, cimg_appname "_%.4u.eps",
                          snap_number++);
            if ((file = cimg::std_fopen(filename, "r")) != 0)
              cimg::fclose(file);
          } while (file);
          (+visu)
              .__draw_text(" Saving EPS snapshot... ", font_size, 0)
              .display(disp);
          LibBoard::Board board;
          (+visu)._draw_object3d(
              &board, zbuffer.fill(0), Xoff + visu._width / 2.f,
              Yoff + visu._height / 2.f, Zoff, rotated_vertices,
              reverse_primitives ? reverse_primitives : primitives, colors,
              opacities, clicked ? nrender_motion : nrender_static,
              _is_double_sided == 1, focale, visu.width() / 2.f + light_x,
              visu.height() / 2.f + light_y, light_z + Zoff, specular_lightness,
              specular_shininess, 1, sprite_scale);
          board.saveEPS(filename);
          (+visu)
              .__draw_text(" Object '%s' saved. ", font_size, 0, filename._data)
              .display(disp);
          disp.set_key(key, false);
          key = 0;
        }
        break;
      case cimg::keyV:
        if (disp.is_keyCTRLLEFT() || disp.is_keyCTRLRIGHT())
        {  // Save object as a .SVG file
          static unsigned int snap_number = 0;
          std::FILE* file;
          do
          {
            cimg_snprintf(filename, filename._width, cimg_appname "_%.4u.svg",
                          snap_number++);
            if ((file = cimg::std_fopen(filename, "r")) != 0)
              cimg::fclose(file);
          } while (file);
          (+visu)
              .__draw_text(" Saving SVG snapshot... ", font_size, 0)
              .display(disp);
          LibBoard::Board board;
          (+visu)._draw_object3d(
              &board, zbuffer.fill(0), Xoff + visu._width / 2.f,
              Yoff + visu._height / 2.f, Zoff, rotated_vertices,
              reverse_primitives ? reverse_primitives : primitives, colors,
              opacities, clicked ? nrender_motion : nrender_static,
              _is_double_sided == 1, focale, visu.width() / 2.f + light_x,
              visu.height() / 2.f + light_y, light_z + Zoff, specular_lightness,
              specular_shininess, 1, sprite_scale);
          board.saveSVG(filename);
          (+visu)
              .__draw_text(" Object '%s' saved. ", font_size, 0, filename._data)
              .display(disp);
          disp.set_key(key, false);
          key = 0;
        }
        break;
#endif
    }
    if (disp.is_resized())
    {
      disp.resize(false);
      visu0 = img.get_resize(disp, 1);
      if (zbuffer) zbuffer.assign(disp.width(), disp.height());
      redraw = true;
    }
    if (!exit_on_anykey && key && key != cimg::keyESC &&
        (key != cimg::keyW ||
         (!disp.is_keyCTRLLEFT() && !disp.is_keyCTRLRIGHT())))
    {
      key = 0;
    }
    auto frame_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start).count();
    // std::cout << duration / 1000. << "\n";
  }
  // if (pose_matrix)
  // {
  //   std::memcpy(pose_matrix, pose._data, 12 * sizeof(float));
  //   pose_matrix[12] = Xoff;
  //   pose_matrix[13] = Yoff;
  //   pose_matrix[14] = Zoff;
  //   pose_matrix[15] = sprite_scale;
  // }
  disp.set_button().set_key(key);

  return img;
}

int main(int argc, const char** argv)
{
  std::cout << argc << "," << argv[0] << ";";
  if (argc < 2)
  {
    std::cout << "Missing map\n";
    return -1;
  }
  auto odr = opendrive::OpenDriveParser::LoadFile(argv[1]);

  // auto topo = odr->GenerateTopology();
  return 0;
}
