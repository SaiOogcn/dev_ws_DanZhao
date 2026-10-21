/*********************************************************************
 *  Copyright (c) 2018-2023 Shenzhen Guangjian Technology Co.,Ltd..  *
 *                     All rights reserved.                          *
 *********************************************************************/

#ifndef DEPTRUM_STREAM_INCLUDE_STREAM_VISUALIZER_H_
#define DEPTRUM_STREAM_INCLUDE_STREAM_VISUALIZER_H_

// #include <string>
// #include "deptrum/common_types.h"
#include "stream_types.h"

namespace deptrum {
namespace stream {

/**
 * Colormap the depth map on default colormap_table and add shading
 *
 * @param[out] color_image Colored depth map, memory needs to be applied externally.
 * @param[in] rows Rows of depth map.
 * @param[in] cols Columns of depth map.
 * @param[in] range_min Minimum mapping range of depth map in millimeter (mm).
 * @param[in] range_mid Middle value of mapping range of depth map in millimeter (mm).
 * @param[in] range_max Maximum mapping range of depth map in millimeter (mm).
 * @param[in] camera_param Camera intrinsic parameters.
 * @param[in] depth_map Depth map to be color mapped.
 * @return Zero on success, error code otherwise.
 */
int STREAM_DLL AddShaderOnDepthMap(uint8_t* color_image,
                                   const int rows,
                                   const int cols,
                                   const int range_min,
                                   const int range_mid,
                                   const int range_max,
                                   const CameraParam& camera_param,
                                   const uint16_t* depth_map);

/**
 * Colormap the depth map on default colormap_table
 *
 * @param[out] colored_depth Colored depth map, memory needs to be applied externally.
 * @param[in] rows Rows of depth map.
 * @param[in] cols Columns of depth map.
 * @param[in] range_min Minimum mapping range of depth map in millimeter (mm).
 * @param[in] range_mid Middle value of mapping range of depth map in millimeter (mm).
 * @param[in] range_max Maximum mapping range of depth map in millimeter (mm).
 * @param[in] depth_map Depth map to be color mapped.
 * @return Zero on success, error code otherwise.
 */
int STREAM_DLL ColorizeDepth(uint8_t* colored_depth,
                             const int rows,
                             const int cols,
                             const int range_min,
                             const int range_mid,
                             const int range_max,
                             const uint16_t* depth_map);

/**
 * Add shading on colored depth map or rgb image aligned with depth map
 * Input color map, depth map and camera internal parameters,
 * and calculate the shadow effect according to the nearest neighbor distance.
 *
 * @param[inout] colored_depth Shaded colordepth map, pixel in bgr888.
 * @param[in] depth_map Depth map to be color mapped.
 * @param[in] rows Rows of depth map.
 * @param[in] cols Columns of depth map.
 * @param[in] camera_param Camera intrinsic parameters.
 * @return Zero on success, error code otherwise.
 */
int STREAM_DLL AddShadingOnColorDepthMap(uint8_t* colored_depth,
                                         const uint16_t* depth_map,
                                         const int rows,
                                         const int cols,
                                         const CameraParam& camera_param);
/**
 * Draw depth map in color scale through histogram equalization
 *
 * @param[out] color_image Colored depth map, memory needs to be applied externally.
 * @param[in] rows Rows of depth map.
 * @param[in] cols Columns of depth map.
 * @param[in] data Depth map to be colormap.
 */
int STREAM_DLL ColorizeDepthByHist(uint8_t* color_image, int rows, int cols, const uint16_t* data);

}  // namespace stream
}  // namespace deptrum

#endif  // DEPTRUM_STREAM_INCLUDE_STREAM_VISUALIZER_H_
