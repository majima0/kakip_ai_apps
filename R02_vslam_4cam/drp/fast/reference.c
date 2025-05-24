#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "TBin_out.h"
#include "pgm_read_write.h"

#define MEASURE_N 10000

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define TRUE 1
#define FALSE 0

#define MAX_CLIP_SIZE 80 * 80
#define MAX_CLIP_KEYPOINTS MAX_CLIP_SIZE
#define MAX_ALL_KEYPOINTS 9000

void makeOffsets(int pixel[25], int rowStride, int patternSize) {
  static const int offsets16[][2] =
      {
          {0, 3}, {1, 3}, {2, 2}, {3, 1}, {3, 0}, {3, -1}, {2, -2}, {1, -3}, {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}};

  static const int offsets12[][2] =
      {
          {0, 2}, {1, 2}, {2, 1}, {2, 0}, {2, -1}, {1, -2}, {0, -2}, {-1, -2}, {-2, -1}, {-2, 0}, {-2, 1}, {-1, 2}};

  static const int offsets8[][2] =
      {
          {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};

  const int(*offsets)[2] = patternSize == 16 ? offsets16 : patternSize == 12 ? offsets12
                                                       : patternSize == 8    ? offsets8
                                                                             : 0;

  int k = 0;
  for (; k < patternSize; k++)
    pixel[k] = offsets[k][0] + offsets[k][1] * rowStride;
  for (; k < 25; k++)
    pixel[k] = pixel[k - patternSize];
}

int cornerScore(const unsigned char* ptr, const int pixel[], int threshold) {
  const int K = 8, N = K * 3 + 1;
  int       k, v     = ptr[0];
  short     d[N];
  for (k = 0; k < N; k++)
    d[k] = (short)(v - ptr[pixel[k]]);

  {
    int a0 = threshold;
    for (k = 0; k < 16; k += 2) {
      int a = MIN((int)d[k + 1], (int)d[k + 2]);
      a     = MIN(a, (int)d[k + 3]);
      if (a <= a0)
        continue;
      a  = MIN(a, (int)d[k + 4]);
      a  = MIN(a, (int)d[k + 5]);
      a  = MIN(a, (int)d[k + 6]);
      a  = MIN(a, (int)d[k + 7]);
      a  = MIN(a, (int)d[k + 8]);
      a0 = MAX(a0, MIN(a, (int)d[k]));
      a0 = MAX(a0, MIN(a, (int)d[k + 9]));
    }

    int b0 = -a0;
    for (k = 0; k < 16; k += 2) {
      int b = MAX((int)d[k + 1], (int)d[k + 2]);
      b     = MAX(b, (int)d[k + 3]);
      b     = MAX(b, (int)d[k + 4]);
      b     = MAX(b, (int)d[k + 5]);
      if (b >= b0)
        continue;
      b = MAX(b, (int)d[k + 6]);
      b = MAX(b, (int)d[k + 7]);
      b = MAX(b, (int)d[k + 8]);

      b0 = MIN(b0, MAX(b, (int)d[k]));
      b0 = MIN(b0, MAX(b, (int)d[k + 9]));
    }

    threshold = -b0 - 1;
  }

  return threshold;
}

unsigned char* alignPtr(unsigned char* ptr, int n) {
  return (unsigned char*)(((size_t)ptr + n - 1) & -n);
}

#define patternSize 16
int FAST_t(unsigned char* img, int rows, int cols, int step, KeyPoint* keypoints, int threshold, int nonmax_suppression) {
  size_t    idx = 0;
  const int K = patternSize / 2, N = patternSize + K + 1;
  int       i, j, k, pixel[25];
  makeOffsets(pixel, step, patternSize);

  threshold = MIN(MAX(threshold, 0), 255);

  unsigned char threshold_tab[512];
  for (i = -255; i <= 255; i++)
    threshold_tab[i + 255] = (unsigned char)(i < -threshold ? 1 : i > threshold ? 2
                                                                                : 0);

  const size_t   size = (cols + 16) * 3 * (sizeof(int) + sizeof(unsigned char)) + 128;
  unsigned char  _buf_ptr[size];
  unsigned char* buf[3];
  buf[0] = _buf_ptr;
  buf[1] = buf[0] + cols;
  buf[2] = buf[1] + cols;
  int* cpbuf[3];
  cpbuf[0] = (int*)alignPtr(buf[2] + cols, sizeof(int)) + 1;
  cpbuf[1] = cpbuf[0] + cols + 1;
  cpbuf[2] = cpbuf[1] + cols + 1;
  memset(buf[0], 0, cols * 3);

  for (i = 3; i < rows - 2; i++) {
    const unsigned char* ptr       = img + i * step + 3;
    unsigned char*       curr      = buf[(i - 3) % 3];
    int*                 cornerpos = cpbuf[(i - 3) % 3];
    memset(curr, 0, cols);
    int ncorners = 0;

    if (i < rows - 3) {
      j = 3;
      for (; j < cols - 3; j++, ptr++) {
        int                  v   = ptr[0];
        const unsigned char* tab = &threshold_tab[0] - v + 255;
        int                  d   = tab[ptr[pixel[0]]] | tab[ptr[pixel[8]]];

        if (d == 0)
          continue;

        d &= tab[ptr[pixel[2]]] | tab[ptr[pixel[10]]];
        d &= tab[ptr[pixel[4]]] | tab[ptr[pixel[12]]];
        d &= tab[ptr[pixel[6]]] | tab[ptr[pixel[14]]];

        if (d == 0)
          continue;

        d &= tab[ptr[pixel[1]]] | tab[ptr[pixel[9]]];
        d &= tab[ptr[pixel[3]]] | tab[ptr[pixel[11]]];
        d &= tab[ptr[pixel[5]]] | tab[ptr[pixel[13]]];
        d &= tab[ptr[pixel[7]]] | tab[ptr[pixel[15]]];

        if (d & 1) {
          int vt = v - threshold, count = 0;

          for (k = 0; k < N; k++) {
            int x = ptr[pixel[k]];
            if (x < vt) {
              if (++count > K) {
                cornerpos[ncorners++] = j;
                if (nonmax_suppression)
                  curr[j] = (unsigned char)cornerScore(ptr, pixel, threshold);
                break;
              }
            } else
              count = 0;
          }
        }

        if (d & 2) {
          int vt = v + threshold, count = 0;

          for (k = 0; k < N; k++) {
            int x = ptr[pixel[k]];
            if (x > vt) {
              if (++count > K) {
                cornerpos[ncorners++] = j;
                if (nonmax_suppression)
                  curr[j] = (unsigned char)cornerScore(ptr, pixel, threshold);
                break;
              }
            } else
              count = 0;
          }
        }
      }
    }

    cornerpos[-1] = ncorners;

    if (i == 3)
      continue;

    const unsigned char* prev  = buf[(i - 4 + 3) % 3];
    const unsigned char* pprev = buf[(i - 5 + 3) % 3];
    cornerpos                  = cpbuf[(i - 4 + 3) % 3];
    ncorners                   = cornerpos[-1];

    for (k = 0; k < ncorners && idx < MAX_CLIP_KEYPOINTS; k++) {
      j         = cornerpos[k];
      int score = prev[j];
      if (!nonmax_suppression ||
          (score > prev[j + 1] && score > prev[j - 1] &&
           score > pprev[j - 1] && score > pprev[j] && score > pprev[j + 1] &&
           score > curr[j - 1] && score > curr[j] && score > curr[j + 1])) {
        KeyPoint kp    = {{(float)j, (float)(i - 1)}, 7.f, -1, (float)score, 0, -1};
        keypoints[idx] = kp;
        idx++;
      }
    }
  }

  return idx;
}

int load_keypoints(char* filename, KeyPoint* dst) {
  char  buf[1024];
  FILE* fp;
  if ((fp = fopen(filename, "rb")) == NULL) {
    printf("failed to open.\n");
    return -1;
  }

  /* read col names */
  if (fgets(buf, 1024, fp) == NULL) {
    printf("failed to read.\n");
    fclose(fp);
    return -1;
  }

  float  x, y, size, angle, response;
  int    octave, class_id;
  size_t i = 0;
  while (fscanf(fp, "%f,%f,%f,%d,%d,%f,%f", &x, &y, &angle, &class_id, &octave, &response, &size) != EOF) {
    KeyPoint kp = {{x, y}, size, angle, response, octave, class_id};
    dst[i]      = kp;
    i++;
  }

  fclose(fp);
  return i;
}

int verify(int level, int frame_id, size_t len, KeyPoint* keypts1, KeyPoint* keypts2) {
  size_t      i;
  const float thres = 1e-3;
  for (i = 0; i < len; i++) {
    KeyPoint p  = keypts1[i];
    KeyPoint tp = keypts2[i];
    if (!(
            abs(p.pt.x - tp.pt.x) < thres &&
            abs(p.pt.y - tp.pt.y) < thres &&
            abs(p.angle - tp.angle) < thres &&
            p.class_id == tp.class_id &&
            p.octave == tp.octave &&
            abs(p.response - tp.response) < thres &&
            abs(p.size - tp.size) < thres)) {
      printf("not equals in %d level in %d frame.\n", level, frame_id);
      printf("keypts1[%ld] == [[%f,%f],%f,%f,%f,%d,%d]\n", i, p.pt.x, p.pt.y, p.size, p.angle, p.response, p.octave, p.class_id);
      printf("keypts2[%ld] == [[%f,%f],%f,%f,%f,%d,%d]\n", i, tp.pt.x, tp.pt.y, tp.size, tp.angle, tp.response, tp.octave, tp.class_id);
      return FALSE;
    }
  }

  return TRUE;
}

int main(int argc, char* argv[]) {
  char*  target;
  size_t frame_id, level;
  int    rows, cols;

  size_t i, j, iy, ix;
  int    k;
  size_t times;
  double sum_time   = 0.0;
  int    keypts_len = 0;

  const size_t BUF_SIZE = 100;
  char         filename[BUF_SIZE];

  unsigned char* src;

  const float W              = 30;
  const int   EDGE_THRESHOLD = 19;

  const int orb_params_ini_fast_thr_ = 20;
  const int orb_params_min_fast_thr  = 7;

  const unsigned int overlap = 6;

  const unsigned int min_border_x = EDGE_THRESHOLD - 3;
  const unsigned int min_border_y = min_border_x;
  unsigned int       max_border_x, max_border_y;

  float width, height;

  size_t num_cols, num_rows;
  int    w_cell, h_cell;

  KeyPoint keypts_to_distribute[MAX_ALL_KEYPOINTS];

  unsigned int image_size[2];

  if (argc < 5 || 6 < argc) {
    printf("Usage: ./fast.exe [orb-slam2|yolo-planar-slam|crowd-slam] <frame number> <level number> <column size> <row size>\n");
    return 1;
  }

  if (argc == 5) {
    target   = "";
    frame_id = atoi(argv[1]);
    level    = atoi(argv[2]);
    cols     = atoi(argv[3]);
    rows     = atoi(argv[4]);
  }
  if (argc == 6) {
    target   = argv[1];
    frame_id = atoi(argv[2]);
    level    = atoi(argv[3]);
    cols     = atoi(argv[4]);
    rows     = atoi(argv[5]);

    if (strcmp(target, "orb-slam2") != 0 && strcmp(target, "yolo-planar-slam") != 0 && strcmp(target, "crowd-slam") != 0) {
      printf("Usage: ./fast.exe [orb-slam2|yolo-planar-slam|crowd-slam] <frame number> <level number> <column size> <row size>\n");
      return 1;
    }
  }

  image_size[0] = cols;
  image_size[1] = rows;

  src = (unsigned char*)malloc(sizeof(unsigned char) * rows * cols);
  memset(src, 0x00, sizeof(unsigned char) * rows * cols);

  max_border_x = cols - EDGE_THRESHOLD + 3;
  max_border_y = rows - EDGE_THRESHOLD + 3;
  width        = max_border_x - min_border_x;
  height       = max_border_y - min_border_y;
  num_cols     = width / W;
  num_rows     = height / W;
  w_cell       = ceil(width / num_cols);
  h_cell       = ceil(height / num_rows);

  sprintf(filename, "testbench/%s/image_pyramid/image_pyramid_%04lu_%lu.pgm", target, frame_id, level);
  if (pgm_read(src, image_size, filename)) {
    printf("failed to read %s\n", filename);
    return 1;
  }

  for (times = 0; times < MEASURE_N; times++) {
    long start = clock();
    keypts_len = 0;
    for (i = 0; i < num_rows; ++i) {
      const unsigned int min_y = min_border_y + i * h_cell;
      if (max_border_y - overlap <= min_y) {
        continue;
      }
      unsigned int max_y = min_y + h_cell + overlap;
      if (max_border_y < max_y) {
        max_y = max_border_y;
      }

      for (j = 0; j < num_cols; ++j) {
        const unsigned int min_x = min_border_x + j * w_cell;
        if (max_border_x - overlap <= min_x) {
          continue;
        }
        unsigned int max_x = min_x + w_cell + overlap;
        if (max_border_x < max_x) {
          max_x = max_border_x;
        }

        const size_t  clip_rows = max_y - min_y;
        const size_t  clip_cols = max_x - min_x;
        unsigned char clipped[MAX_CLIP_SIZE];
        for (iy = min_y; iy <= max_y; iy++) {
          size_t row_offset     = iy * cols;
          size_t clipped_offset = (iy - min_y) * clip_cols;
          for (ix = min_x; ix <= max_x; ix++) {
            clipped[clipped_offset + (ix - min_x)] = src[row_offset + ix];
          }
        }

        KeyPoint keypts_in_cell[MAX_CLIP_KEYPOINTS];
        int      len = FAST_t(clipped, clip_rows, clip_cols, clip_cols, keypts_in_cell, orb_params_ini_fast_thr_, TRUE);
        if (len <= 0) {
          len = FAST_t(clipped, clip_rows, clip_cols, clip_cols, keypts_in_cell, orb_params_min_fast_thr, TRUE);
        }

        for (k = 0; k < len; k++) {
          KeyPoint keypt = keypts_in_cell[k];
          keypt.pt.x += j * w_cell;
          keypt.pt.y += i * h_cell;
          keypts_to_distribute[keypts_len] = keypt;
          keypts_len++;
        }
      }
    }
    long end = clock();
    sum_time += (double)(end - start) / CLOCKS_PER_SEC;
  }

  sprintf(filename, "testbench/%s/keypts_to_distribute/keypts_to_distribute_%04lu_%lu.csv", target, frame_id, level);
  KeyPoint ref[MAX_ALL_KEYPOINTS];
  int      ref_len = load_keypoints(filename, ref);
  if (ref_len == -1) {
    printf("failed to load %s.\n", filename);
    return 1;
  }

  if (keypts_len != ref_len) {
    printf("keypts_len != ref_len\n");
    printf("%d != %d\n", keypts_len, ref_len);
  }

  verify(level, frame_id, ref_len, keypts_to_distribute, ref);

  char benchname[100];
  sprintf(benchname, "testbench/%s/fast_%04lu_%lu", target, frame_id, level);
  TBin_out(".", src, keypts_to_distribute, image_size, keypts_len, 0x100000, 0x2000000, 0x3000000, benchname);

  printf("Average time of %u times : %lf[sec]\n", MEASURE_N, sum_time / MEASURE_N);

  free(src);

  return 0;
}
