/*
 * vision.cpp
 *
 *  Created on: May 14, 2013
 *      Author: cforster
 */

#include <vikit/vision.h>

#if __SSE2__
# include <emmintrin.h>
#elif __ARM_NEON__
# include <arm_neon.h>
#endif

namespace vk {

#ifdef __SSE2__
void halfSampleSSE2(const unsigned char* in, unsigned char* out, int w, int h)
{
  const unsigned long long mask[2] = {0x00FF00FF00FF00FFull, 0x00FF00FF00FF00FFull};
  const unsigned char* nextRow = in + w;
  __m128i m = _mm_loadu_si128((const __m128i*)mask);
  int sw = w >> 4;
  int sh = h >> 1;
  for (int i=0; i<sh; i++)
  {
    for (int j=0; j<sw; j++)
    {
      __m128i here = _mm_load_si128((const __m128i*)in);
      __m128i next = _mm_load_si128((const __m128i*)nextRow);
      here = _mm_avg_epu8(here,next);
      next = _mm_and_si128(_mm_srli_si128(here,1), m);
      here = _mm_and_si128(here,m);
      here = _mm_avg_epu16(here, next);
      _mm_storel_epi64((__m128i*)out, _mm_packus_epi16(here,here));
      in += 16;
      nextRow += 16;
      out += 8;
    }
    in += w;
    nextRow += w;
  }
}
#endif 

#ifdef __ARM_NEON__
void halfSampleNEON( const cv::Mat& in, cv::Mat& out )
{
  for( int y = 0; y < in.rows; y += 2)
  {
    const uint8_t * in_top = in.data + y*in.cols;
    const uint8_t * in_bottom = in.data + (y+1)*in.cols;
    uint8_t * out_data = out.data + (y >> 1)*out.cols;
    for( int x = in.cols; x > 0 ; x-=16, in_top += 16, in_bottom += 16, out_data += 8)
    {
      uint8x8x2_t top  = vld2_u8( (const uint8_t *)in_top );
      uint8x8x2_t bottom = vld2_u8( (const uint8_t *)in_bottom );
      uint16x8_t sum = vaddl_u8( top.val[0], top.val[1] );
      sum = vaddw_u8( sum, bottom.val[0] );
      sum = vaddw_u8( sum, bottom.val[1] );
      uint8x8_t final_sum = vshrn_n_u16(sum, 2);
      vst1_u8(out_data, final_sum);
    }
  }
}
#endif


void
halfSample(const cv::Mat& in, cv::Mat& out)
{
  assert( in.rows/2==out.rows && in.cols/2==out.cols);
  assert( in.type()==CV_8U && out.type()==CV_8U);

#ifdef __SSE2__
  if(aligned_mem::is_aligned16(in.data) && aligned_mem::is_aligned16(out.data) && ((in.cols % 16) == 0))
  {
    halfSampleSSE2(in.data, out.data, in.cols, in.rows);
    return;
  }
#endif 
#ifdef __ARM_NEON__ 
  if( (in.cols % 16) == 0 )
  {
    halfSampleNEON(in, out);
    return;
  }
#endif

  const int stride = in.step.p[0];
  uint8_t* top = (uint8_t*) in.data;
  uint8_t* bottom = top + stride;
  uint8_t* end = top + stride*in.rows;
  const int out_width = out.cols;
  uint8_t* p = (uint8_t*) out.data;
  while (bottom < end)
  {
    for (int j=0; j<out_width; j++)
    {
      *p = static_cast<uint8_t>( (uint16_t (top[0]) + top[1] + bottom[0] + bottom[1])/4 );
      p++;
      top += 2;
      bottom += 2;
    }
    top += stride;
    bottom += stride;
  }
}


float
shiTomasiScore(const cv::Mat& img, int u, int v)
{
  assert(img.type() == CV_8UC1);

  float dXX = 0.0;
  float dYY = 0.0;
  float dXY = 0.0;
  const int halfbox_size = 4;
  const int box_size = 2*halfbox_size;
  const int box_area = box_size*box_size;
  const int x_min = u-halfbox_size;
  const int x_max = u+halfbox_size;
  const int y_min = v-halfbox_size;
  const int y_max = v+halfbox_size;

  if(x_min < 1 || x_max >= img.cols-1 || y_min < 1 || y_max >= img.rows-1)
    return 0.0; // patch is too close to the boundary

  const int stride = img.step.p[0];
  for( int y=y_min; y<y_max; ++y )
  {
    const uint8_t* ptr_left   = img.data + stride*y + x_min - 1;
    const uint8_t* ptr_right  = img.data + stride*y + x_min + 1;
    const uint8_t* ptr_top    = img.data + stride*(y-1) + x_min;
    const uint8_t* ptr_bottom = img.data + stride*(y+1) + x_min;
    for(int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom)
    {
      float dx = *ptr_right - *ptr_left;
      float dy = *ptr_bottom - *ptr_top;
      dXX += dx*dx;
      dYY += dy*dy;
      dXY += dx*dy;
    }
  }

  // Find and return smaller eigenvalue:
  dXX = dXX / (2.0 * box_area);
  dYY = dYY / (2.0 * box_area);
  dXY = dXY / (2.0 * box_area);
  return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
}

}


