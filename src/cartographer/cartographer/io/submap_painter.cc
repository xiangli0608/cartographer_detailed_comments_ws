/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/io/submap_painter.h"

#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"

namespace cartographer {
namespace io {
namespace {

Eigen::Affine3d ToEigen(const ::cartographer::transform::Rigid3d& rigid3) {
  return Eigen::Translation3d(rigid3.translation()) * rigid3.rotation();
}

/**
 * @brief 使用传入的函数对传入的submap_slice进行处理
 * 
 * @param[in] scale 分辨率的倒数
 * @param[in] submaps 需要处理的submap_slice
 * @param[in] cr 当前画布
 * @param[in] draw_callback 传入的函数
 */
void CairoPaintSubmapSlices(
    const double scale,
    const std::map<::cartographer::mapping::SubmapId, SubmapSlice>& submaps,
    cairo_t* cr, std::function<void(const SubmapSlice&)> draw_callback) {
  // 进行缩放
  cairo_scale(cr, scale, scale);

  for (auto& pair : submaps) {
    const auto& submap_slice = pair.second;
    if (submap_slice.surface == nullptr) {
      return;
    }
    const Eigen::Matrix4d homo =
        ToEigen(submap_slice.pose * submap_slice.slice_pose).matrix();

    // 保存当前cairo画布
    cairo_save(cr);

    cairo_matrix_t matrix;
    cairo_matrix_init(&matrix, homo(1, 0), homo(0, 0), -homo(1, 1), -homo(0, 1),
                      homo(0, 3), -homo(1, 3));
    // 进行平移
    cairo_transform(cr, &matrix);

    const double submap_resolution = submap_slice.resolution;
    // 进行缩放
    cairo_scale(cr, submap_resolution, submap_resolution);

    // Invokes caller's callback to utilize slice data in global cooridnate
    // frame. e.g. finds bounding box, paints slices.
    // 调用传入的函数进行处理
    draw_callback(submap_slice);

    // 恢复上一次保存的cairo画布
    cairo_restore(cr);
  }
}

bool Has2DGrid(const mapping::proto::Submap& submap) {
  return submap.has_submap_2d() && submap.submap_2d().has_grid();
}

bool Has3DGrids(const mapping::proto::Submap& submap) {
  return submap.has_submap_3d() &&
         submap.submap_3d().has_low_resolution_hybrid_grid() &&
         submap.submap_3d().has_high_resolution_hybrid_grid();
}

}  // namespace

/**
 * @brief 绘制栅格地图的cairo图像
 * 
 * @param[in] submaps 地图图片
 * @param[in] resolution 地图分辨率
 * @return PaintSubmapSlicesResult 
 */
PaintSubmapSlicesResult PaintSubmapSlices(
    const std::map<::cartographer::mapping::SubmapId, SubmapSlice>& submaps,
    const double resolution) {
  Eigen::AlignedBox2f bounding_box;

  {
    // 创建指向contexts的指针
    auto surface = MakeUniqueCairoSurfacePtr(
        cairo_image_surface_create(kCairoFormat, 1, 1));
    // 将上下文与绘制表面绑定
    auto cr = MakeUniqueCairoPtr(cairo_create(surface.get()));

    const auto update_bounding_box = [&bounding_box, &cr](double x, double y) {
      cairo_user_to_device(cr.get(), &x, &y);
      bounding_box.extend(Eigen::Vector2f(x, y));
    };

    // 确定bounding_box
    CairoPaintSubmapSlices(
        1. / resolution, submaps, cr.get(),
        [&update_bounding_box](const SubmapSlice& submap_slice) {
          update_bounding_box(0, 0);
          update_bounding_box(submap_slice.width, 0);
          update_bounding_box(0, submap_slice.height);
          update_bounding_box(submap_slice.width, submap_slice.height);
        });
  }

  const int kPaddingPixel = 5;
  const Eigen::Array2i size(
      std::ceil(bounding_box.sizes().x()) + 2 * kPaddingPixel,
      std::ceil(bounding_box.sizes().y()) + 2 * kPaddingPixel);
  const Eigen::Array2f origin(-bounding_box.min().x() + kPaddingPixel,
                              -bounding_box.min().y() + kPaddingPixel);

  // 要返回的结果
  auto surface = MakeUniqueCairoSurfacePtr(
      cairo_image_surface_create(kCairoFormat, size.x(), size.y()));
  
  {
    auto cr = MakeUniqueCairoPtr(cairo_create(surface.get()));
    // 设置颜色
    cairo_set_source_rgba(cr.get(), 0.5, 0.0, 0.0, 1.);
    // 绘制
    cairo_paint(cr.get());
    // 进行平移
    cairo_translate(cr.get(), origin.x(), origin.y());

    // 根据传入的数据进行图片的绘制
    CairoPaintSubmapSlices(1. / resolution, submaps, cr.get(),
                           [&cr](const SubmapSlice& submap_slice) {
                             // 从surface读取图形数据
                             cairo_set_source_surface(
                                 cr.get(), submap_slice.surface.get(), 0., 0.);
                             // 绘制图形
                             cairo_paint(cr.get());
                           });
    cairo_surface_flush(surface.get());
  }
  
  return PaintSubmapSlicesResult(std::move(surface), origin);
}

void FillSubmapSlice(
    const ::cartographer::transform::Rigid3d& global_submap_pose,
    const ::cartographer::mapping::proto::Submap& proto,
    SubmapSlice* const submap_slice,
    mapping::ValueConversionTables* conversion_tables) {
  ::cartographer::mapping::proto::SubmapQuery::Response response;
  ::cartographer::transform::Rigid3d local_pose;
  if (proto.has_submap_3d()) {
    mapping::Submap3D submap(proto.submap_3d());
    local_pose = submap.local_pose();
    submap.ToResponseProto(global_submap_pose, &response);
  } else {
    ::cartographer::mapping::Submap2D submap(proto.submap_2d(),
                                             conversion_tables);
    local_pose = submap.local_pose();
    submap.ToResponseProto(global_submap_pose, &response);
  }
  submap_slice->pose = global_submap_pose;

  auto& texture_proto = response.textures(0);
  const SubmapTexture::Pixels pixels = UnpackTextureData(
      texture_proto.cells(), texture_proto.width(), texture_proto.height());
  submap_slice->width = texture_proto.width();
  submap_slice->height = texture_proto.height();
  submap_slice->resolution = texture_proto.resolution();
  submap_slice->slice_pose =
      ::cartographer::transform::ToRigid3(texture_proto.slice_pose());
  submap_slice->surface =
      DrawTexture(pixels.intensity, pixels.alpha, texture_proto.width(),
                  texture_proto.height(), &submap_slice->cairo_data);
}

void DeserializeAndFillSubmapSlices(
    ProtoStreamDeserializer* deserializer,
    std::map<mapping::SubmapId, SubmapSlice>* submap_slices,
    mapping::ValueConversionTables* conversion_tables) {
  std::map<mapping::SubmapId, transform::Rigid3d> submap_poses;
  for (const auto& trajectory : deserializer->pose_graph().trajectory()) {
    for (const auto& submap : trajectory.submap()) {
      submap_poses[mapping::SubmapId(trajectory.trajectory_id(),
                                     submap.submap_index())] =
          transform::ToRigid3(submap.pose());
    }
  }
  mapping::proto::SerializedData proto;
  while (deserializer->ReadNextSerializedData(&proto)) {
    if (proto.has_submap() &&
        (Has2DGrid(proto.submap()) || Has3DGrids(proto.submap()))) {
      const auto& submap = proto.submap();
      const mapping::SubmapId id{submap.submap_id().trajectory_id(),
                                 submap.submap_id().submap_index()};
      FillSubmapSlice(submap_poses.at(id), submap, &(*submap_slices)[id],
                      conversion_tables);
    }
  }
}

/**
 * @brief 将地图栅格数据进行解压
 * 
 * @param[in] compressed_cells 压缩后的地图栅格数据
 * @param[in] width 地图的宽
 * @param[in] height 地图的高
 * @return SubmapTexture::Pixels 解压后的地图栅格数据
 */
SubmapTexture::Pixels UnpackTextureData(const std::string& compressed_cells,
                                        const int width, const int height) {
  SubmapTexture::Pixels pixels;
  std::string cells;
  // 将压缩后的地图栅格数据 解压成 字符串
  ::cartographer::common::FastGunzipString(compressed_cells, &cells);
  
  const int num_pixels = width * height;
  CHECK_EQ(cells.size(), 2 * num_pixels);
  pixels.intensity.reserve(num_pixels);
  pixels.alpha.reserve(num_pixels);

  // 填充数据
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      // 地图数据存了intensity与alpha两个值
      pixels.intensity.push_back(cells[(i * width + j) * 2]);
      pixels.alpha.push_back(cells[(i * width + j) * 2 + 1]);
    }
  }
  return pixels;
}

/**
 * @brief 指向新创建的图像的指针
 * 
 * @param[in] intensity 地图栅格数据
 * @param[in] alpha 地图栅格的透明度
 * @param[in] width 地图的宽
 * @param[in] height 地图的高
 * @param[out] cairo_data 4字节的值, 左边3个字节分别存储了alpha_value intensity_value 与 observed
 * @return UniqueCairoSurfacePtr 指向新创建的图像的指针
 */
UniqueCairoSurfacePtr DrawTexture(const std::vector<char>& intensity,
                                  const std::vector<char>& alpha,
                                  const int width, const int height,
                                  std::vector<uint32_t>* const cairo_data) {
  CHECK(cairo_data->empty());

  // Properly dealing with a non-common stride would make this code much more
  // complicated. Let's check that it is not needed.
  const int expected_stride = 4 * width;
  CHECK_EQ(expected_stride, cairo_format_stride_for_width(kCairoFormat, width));
  
  // 对cairo_data进行填充
  for (size_t i = 0; i < intensity.size(); ++i) {
    // We use the red channel to track intensity information. The green
    // channel we use to track if a cell was ever observed.
    // 使用红色通道来跟踪强度信息 绿色通道来追踪栅格是否被观察到
    const uint8_t intensity_value = intensity.at(i);
    const uint8_t alpha_value = alpha.at(i);
    const uint8_t observed =
        (intensity_value == 0 && alpha_value == 0) ? 0 : 255;
    // tag: 这里需要确认一下
    cairo_data->push_back((alpha_value << 24) |     // 第一字节 存储透明度
                          (intensity_value << 16) | // 第二字节 存储栅格值
                          (observed << 8) |         // 第三字节 存储是否被更新过
                          0);                       // 第四字节 始终为0
  }

  // c++11: reinterpret_cast 用于进行各种不同类型的指针之间、不同类型的引用之间以及指针和能容纳指针的整数类型之间的转换

  // MakeUniqueCairoSurfacePtr 生成一个指向cairo_surface_t数据的指针
  auto surface = MakeUniqueCairoSurfacePtr(
    // cairo_image_surface_create_for_data: 根据提供的像素数据创建surface, 返回指向新创建的surface的指针
    cairo_image_surface_create_for_data(
      reinterpret_cast<unsigned char*>(cairo_data->data()), kCairoFormat, width,
      height, expected_stride) );
        
  CHECK_EQ(cairo_surface_status(surface.get()), CAIRO_STATUS_SUCCESS)
      << cairo_status_to_string(cairo_surface_status(surface.get()));
  return surface;
}

}  // namespace io
}  // namespace cartographer
