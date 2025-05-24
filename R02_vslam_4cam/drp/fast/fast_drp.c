#if defined(DRP6)
#include "spa4_if.h"
#else
#include "spa3_if.h"
#endif

#define FIXED_POINT16 (65536.0f)  // 2^16

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define WORDS_PER_VMU 512
#define WORDS_PER_HMU 8192
#define WORDS_PER_VMEM (WORDS_PER_VMU / 2)
#define MAX_KEYPOINTS (WORDS_PER_VMEM * 8)  // size of vMEM16x8_RAM
#define IMG_WIDTH_MAX 1920

#define W 30
#define EDGE_THRESHOLD 19

#define OVERLAP 6
#define MAX_CLIP_SIZE 60

#define NUM_HMU 8  // for DMA read line buffer

#define patternSize 16
#define CONV_RANGE 7
#define CONV_CENTER 3

typedef struct {
  unsigned short x;
  unsigned short y;
} Point2us;

typedef struct {
  Point2us pt;
  short    response;
} KeyPoint;

// patternSize = 16
const short pixel[25][2] = {
    {3, 6}, {4, 6}, {5, 5}, {6, 4}, {6, 3}, {6, 2}, {5, 1}, {4, 0}, {3, 0}, {2, 0}, {1, 1}, {0, 2}, {0, 3}, {0, 4}, {1, 5}, {2, 6}, {3, 6}, {4, 6}, {5, 5}, {6, 4}, {6, 3}, {6, 2}, {5, 1}, {4, 0}, {3, 0}};

/* Cyber func = operator */
signed short MAXS_(signed short in0, signed short in1) {
  return MAX(in0, in1);
}

/* Cyber func = operator */
unsigned short MAX_(unsigned short in0, unsigned short in1) {
  return MAX(in0, in1);
}

/* Cyber func = inline */
unsigned short max4(unsigned short in0, unsigned short in1, unsigned short in2, unsigned short in3) {
  unsigned short max[3];
  max[0] = MAX(in0, in1);
  max[1] = MAX(in2, in3);
  max[2] = MAX(max[0], max[1]);
  return max[2];
}

/* Cyber func = inline */
unsigned short min4(unsigned short in0, unsigned short in1, unsigned short in2, unsigned short in3) {
  unsigned short min[3];
  min[0] = MIN(in0, in1);
  min[1] = MIN(in2, in3);
  min[2] = MIN(min[0], min[1]);
  return min[2];
}

short corner_score_brighter(const unsigned short mtxpix[][CONV_RANGE]) {
  const unsigned short K = 8;
  const unsigned short N = K * 3 + 1;
  unsigned short       d[N];

  unsigned short a[8];
  short          a0;

  const unsigned short v = mtxpix[CONV_CENTER][CONV_CENTER];

  unsigned short mins[16 + 4];
  unsigned short max_ends[16 + 4];

  for (unsigned short k = 0; k < N; k++)
    d[k] = mtxpix[pixel[k][1]][pixel[k][0]];

  for (unsigned short k = 0; k < 16; k++) {
    mins[k] = min4(d[k], d[k + 1], d[k + 2], d[k + 3]);
  }

  for (unsigned short k = 0; k < 16; k++) {
    max_ends[k] = MAX(d[k], d[k + 9]);
  }

  a[0] = MIN(max_ends[0], MIN(mins[1], mins[5]));
  a[1] = MIN(max_ends[4], MIN(mins[5], mins[9]));
  a[2] = MIN(max_ends[2], MIN(mins[3], mins[7]));
  a[3] = MIN(max_ends[6], MIN(mins[7], mins[11]));
  a[4] = MIN(max_ends[8], MIN(mins[9], mins[13]));
  a[5] = MIN(max_ends[10], MIN(mins[11], mins[15]));
  a[6] = MIN(max_ends[12], MIN(mins[13], mins[1]));
  a[7] = MIN(max_ends[14], MIN(mins[15], mins[3]));

  a0 =
      max4(
          MAX(a[0], a[1]),
          MAX(a[2], a[3]),
          MAX(a[4], a[5]),
          MAX(a[6], a[7])) -
      v;

  return a0 - 1;
}

short corner_score_darker(const unsigned short mtxpix[][CONV_RANGE]) {
  const unsigned short K = 8;
  const unsigned short N = K * 3 + 1;
  unsigned short       d[N];

  unsigned short b[8];
  short          b0;

  const unsigned short v = mtxpix[CONV_CENTER][CONV_CENTER];

  unsigned short maxs[16 + 4];
  unsigned short min_ends[16 + 4];

  for (unsigned short k = 0; k < N; k++)
    d[k] = mtxpix[pixel[k][1]][pixel[k][0]];

  for (unsigned short k = 0; k < 16; k++) {
    maxs[k] = MAXS_(
        MAX_(d[k], d[k + 1]),
        MAX_(d[k + 2], d[k + 3]));
  }

  for (unsigned short k = 0; k < 16; k++) {
    min_ends[k] = MIN(d[k], d[k + 9]);
  }

  b[0] = MAX_(min_ends[0], MAX_(maxs[1], maxs[5]));
  b[1] = MAX_(min_ends[2], MAX_(maxs[3], maxs[7]));
  b[2] = MAX_(min_ends[4], MAX_(maxs[5], maxs[9]));
  b[3] = MAX_(min_ends[6], MAX_(maxs[7], maxs[11]));
  b[4] = MAX_(min_ends[8], MAX_(maxs[9], maxs[13]));
  b[5] = MAX_(min_ends[10], MAX_(maxs[11], maxs[15]));
  b[6] = MAX_(min_ends[12], MAX_(maxs[13], maxs[1]));
  b[7] = MAX_(min_ends[14], MAX_(maxs[15], maxs[3]));

  b0 =
      v - min4(
              MIN(b[0], b[1]),
              MIN(b[2], b[3]),
              MIN(b[4], b[5]),
              MIN(b[6], b[7]));

  return b0 - 1;
}

unsigned short FAST_thresholds(
    unsigned int         addr,
    const unsigned short max_y,
    const unsigned short min_y,
    const unsigned short max_x,
    const unsigned short min_x,
    unsigned short       clip_rows,
    unsigned short       clip_cols,
    unsigned short       cols,
    KeyPoint*            keypoints,
    short                threshold_ini,
    short                threshold_min,
    bool                 nonmax_suppression,
    unsigned short*      len) {
  unsigned short idx       = 0;
  unsigned short start_pos = 0;

  bool ini_exist = false;

  const short nms_thres_ini = threshold_ini - 1;
  const short nms_thres_min = threshold_min - 1;

  const unsigned int   y_offset    = min_y * cols;
  const unsigned short inside_rows = clip_rows + 1;
  const unsigned short border_rows = clip_rows + 2;
  const unsigned short inside_cols = clip_cols - 3;

  unsigned longlong din1, din3;
  unsigned longlong dout;

  short brighter_buf[MAX_CLIP_SIZE + 4][MAX_CLIP_SIZE + 4] /* Cyber sig2mu = hMEM16x1_RAM */;

  short          pprev[MAX_CLIP_SIZE + 4] /* Cyber sig2mu = vMEM16x1_RAM, hazard_avoid_sche = YES */;
  short          prev[MAX_CLIP_SIZE + 4] /* Cyber sig2mu = vMEM16x1_RAM */;
  short          curr[MAX_CLIP_SIZE + 4] /* Cyber sig2mu = vMEM16x1_RAM */;
  unsigned short inbuf[CONV_RANGE][MAX_CLIP_SIZE + 4] /* Cyber array2mem_file = 2 */;
  unsigned short mtxpix[CONV_RANGE][CONV_RANGE] /* Cyber array = REG */;
  unsigned short nextpix[CONV_RANGE] /* Cyber array = REG */;

  const unsigned short read_width  = max_x - min_x + 1;
  const unsigned short read_height = max_y - min_y;

  CmdDinDmaStride(STP_CH1, STP_WIDTH1, STP_NOWAIT, addr + (y_offset + min_x), read_width, read_height, cols);

  for (unsigned short j = 0; j < MAX_CLIP_SIZE + 4; j++) prev[j] = 0;

  /* Cyber folding = 1 */
  for (unsigned short i = 0; i < border_rows; i++) {
    /* Cyber folding = 1, unnesting = YES */
    for (unsigned short j = 0; j < MAX_CLIP_SIZE + 4; j++) {
      for (unsigned short ii = 0; ii < CONV_RANGE; ii++) {
        nextpix[ii] = inbuf[ii][j];
      }

      brighter_buf[i][j] = corner_score_brighter(mtxpix);

      // shift left
      for (unsigned short ii = 0; ii < CONV_RANGE; ii++) {
        for (unsigned short jj = 0; jj < CONV_RANGE - 1; jj++) {
          mtxpix[ii][jj] = mtxpix[ii][jj + 1];
        }
        mtxpix[ii][CONV_RANGE - 1] = nextpix[ii];
      }

      // shift up
      if (j < MAX_CLIP_SIZE) {
        for (unsigned short ii = 0; ii < CONV_RANGE - 1; ii++) {
          inbuf[ii][j] = nextpix[ii + 1];
        }
        if (j < read_width && i <= (unsigned short)(clip_rows - (CONV_RANGE - 6))) {
          din1                     = ReadFifo(STP_CH1);
          inbuf[CONV_RANGE - 1][j] = U64ToU8(din1, 0);
        } else {
          inbuf[CONV_RANGE - 1][j] = 0;
        }
      }
    }
  }

  CmdDinDmaStride(STP_CH1, STP_WIDTH1, STP_NOWAIT, addr + (y_offset + min_x), read_width, read_height, cols);

  /* Cyber folding = 1 */
  for (unsigned short i = 0; i < border_rows; i++) {
    short nms2_list_ini[3];
    short nms3_list_ini[3];
    short nms2_list_min[3];
    short nms3_list_min[3];
    short prev_left;
    short score;
    short prev_right;

    /* Cyber folding = 1, unnesting = YES */
    for (unsigned short j = 0; j < MAX_CLIP_SIZE + 4; j++) {
      short score_brighter;
      short score_darker;
      short score_temp;

      score_brighter = brighter_buf[i][j];

      for (unsigned short ii = 0; ii < CONV_RANGE; ii++) {
        nextpix[ii] = inbuf[ii][j];
      }

      score_darker = corner_score_darker(mtxpix);
      score_temp   = MAX(score_brighter, score_darker);
      if (clip_cols < j) score_temp = 0;
      if (i == inside_rows) score_temp = 0;
      curr[j] = score_temp;

      // shift left
      for (unsigned short ii = 0; ii < CONV_RANGE; ii++) {
        for (unsigned short jj = 0; jj < CONV_RANGE - 1; jj++) {
          mtxpix[ii][jj] = mtxpix[ii][jj + 1];
        }
        mtxpix[ii][CONV_RANGE - 1] = nextpix[ii];
      }

      // shift up
      if (j < MAX_CLIP_SIZE) {
        for (unsigned short ii = 0; ii < CONV_RANGE - 1; ii++) {
          inbuf[ii][j] = nextpix[ii + 1];
        }
        if (j < read_width && i <= (unsigned short)(clip_rows - (CONV_RANGE - 6))) {
          din1                     = ReadFifo(STP_CH1);
          inbuf[CONV_RANGE - 1][j] = U64ToU8(din1, 0);
        } else {
          inbuf[CONV_RANGE - 1][j] = 0;
        }
      }

      if (j == 5) {
        nms3_list_min[2] = 255;
        nms3_list_ini[2] = 255;
      }
      if (j == 6) {
        nms3_list_min[2] = 0;
        nms3_list_ini[2] = 0;
      }

      prev_right = prev[j];

      if (7 < i && 7 <= j) {
        short thres_min;
        short thres_ini;

        nms2_list_min[2] = MAX(pprev[j], MAX(nms_thres_min, curr[j]));
        nms3_list_min[2] = MAX(nms2_list_min[2], prev_right);

        nms2_list_ini[2] = MAX(nms2_list_min[2], nms_thres_ini);
        nms3_list_ini[2] = MAX(nms2_list_ini[2], prev_right);

        thres_min = MAX(nms3_list_min[0], MAX(nms2_list_min[1], nms3_list_min[2]));
        thres_ini = MAX(nms3_list_ini[0], MAX(nms2_list_ini[1], nms3_list_ini[2]));

        if (!ini_exist && score > thres_min) {
          KeyPoint kp;
          kp.pt.x        = j;
          kp.pt.y        = i;
          kp.response    = score;
          keypoints[idx] = kp;
          idx++;
        }

        if (!nonmax_suppression || score > thres_ini) {
          KeyPoint kp;

          kp.pt.x        = j;
          kp.pt.y        = i;
          kp.response    = score;
          keypoints[idx] = kp;

          if (!ini_exist) start_pos = idx;

          idx++;

          ini_exist = true;
        }
      }

      // shift left
      // nms2_list_ini[0] = nms2_list_ini[1];
      nms2_list_ini[1] = nms2_list_ini[2];
      nms3_list_ini[0] = nms3_list_ini[1];
      nms3_list_ini[1] = nms3_list_ini[2];

      // nms2_list_min[0] = nms2_list_min[1];
      nms2_list_min[1] = nms2_list_min[2];
      nms3_list_min[0] = nms3_list_min[1];
      nms3_list_min[1] = nms3_list_min[2];

      if (0 <= j - 2 && 7 <= i) {
        pprev[j - 2] = prev_left;  // == prev[j - 2]
        prev[j - 2]  = curr[j - 2];
      }

      prev_left = score;
      score     = prev_right;
    }
  }

  *len = idx;
  return start_pos;
}

void FAST_all(
    unsigned int   addr,
    unsigned int   addw_keypoints,
    unsigned int   addw_num,
    unsigned short rows,
    unsigned short cols) {
  unsigned longlong dout0;
  unsigned short    keypts_len = 0;

  const short orb_params_ini_fast_thr_ = 20;
  const short orb_params_min_fast_thr  = 7;

  const unsigned short min_border_x = EDGE_THRESHOLD - 3;
  const unsigned short min_border_y = EDGE_THRESHOLD - 3;
  const unsigned short max_border_x = cols - (EDGE_THRESHOLD - 3);
  const unsigned short max_border_y = rows - (EDGE_THRESHOLD - 3);

  const unsigned short width  = max_border_x - min_border_x;
  const unsigned short height = max_border_y - min_border_y;

  const unsigned short num_cols = width / W;
  const unsigned short num_rows = height / W;

  // Ceil
#if defined(DRP6)
  // (short_float)0.999511718750 == U16ToFP16(0x3bff)
  const short_float    fraction = U16ToFP16(0b0011101111011111);  // 0.9838867188
  const unsigned short w_cell   = (unsigned short)((((short_float)width) / num_cols) + fraction);
  const unsigned short h_cell   = (unsigned short)((((short_float)height) / num_rows) + fraction);
#else
  // (short_float)0.5 == U16ToFP16(0x3800)
  const short_float    half   = U16ToFP16(0b0011011110000000);  // 0.468750
  const unsigned short w_cell = (unsigned short)((((short_float)width) / num_cols) + half);
  const unsigned short h_cell = (unsigned short)((((short_float)height) / num_rows) + half);
#endif

  CmdDoutDma(STP_CH1, STP_WIDTH2, STP_WAIT, addw_num, 2);

  for (unsigned short i = 0; i < num_rows; ++i) {
    const unsigned short min_y = min_border_y + i * h_cell;
    unsigned short       max_y = min_y + h_cell + OVERLAP;
    if (max_border_y - OVERLAP <= min_y) {
      continue;
    }
    if (max_border_y < max_y) {
      max_y = max_border_y;
    }

    for (unsigned short j = 0; j < num_cols; ++j) {
      const unsigned short min_x     = min_border_x + j * w_cell;
      unsigned short       max_x     = min_x + w_cell + OVERLAP;
      const unsigned short clip_rows = max_y - min_y;
      unsigned short       clip_cols;
      KeyPoint             keypts_in_cell[MAX_KEYPOINTS] /* Cyber sig2mu = vMEM16x8_RAM */;
      unsigned short       start_pos;
      unsigned short       len;

      if (max_border_x - OVERLAP <= min_x) {
        continue;
      }
      if (max_border_x < max_x) {
        max_x = max_border_x;
      }

      clip_cols = max_x - min_x;

      /*
      len = FAST_t(addr, max_y, min_y, max_x, min_x, clip_rows, clip_cols, cols, keypts_in_cell, orb_params_ini_fast_thr_, true);
      if (len <= 0) {
        len = FAST_t(addr, max_y, min_y, max_x, min_x, clip_rows, clip_cols, cols, keypts_in_cell, orb_params_min_fast_thr, true);
      }
      */

      start_pos = FAST_thresholds(addr,
                                  max_y, min_y, max_x, min_x,
                                  clip_rows, clip_cols, cols,
                                  keypts_in_cell,
                                  orb_params_ini_fast_thr_, orb_params_min_fast_thr, true,
                                  &len);

      if (len == 0) continue;

      CmdDoutDmaStride(STP_CH0, STP_WIDTH5, STP_WAIT, addw_keypoints + 8 * keypts_len, 5, len - start_pos, 8);
      for (unsigned short k = start_pos; k < len; k++) {
        KeyPoint keypt;
        keypt.pt.x     = keypts_in_cell[k].pt.x - 5;
        keypt.pt.y     = keypts_in_cell[k].pt.y - 5;
        keypt.response = keypts_in_cell[k].response;
        keypt.pt.x += j * w_cell;
        keypt.pt.y += i * h_cell;
        // keypts_to_distribute[keypts_len] = keypt;
        dout0 = U16ToU64(0, keypt.response, keypt.pt.y, keypt.pt.x);
        WriteFifo(STP_CH0, dout0);
        keypts_len++;
      }
    }
  }

  dout0 = U16ToU64(0, 0, 0, keypts_len);
  WriteFifo(STP_CH1, dout0);

  return;
}

process drp() {
  unsigned int      addr, addw_keypoints, addw_num;
  unsigned int      rows, cols;
  unsigned longlong din;

  /* ディスクリプタ読み出し */
  din            = ReadFifo(STP_CH0);
  addr           = U64ToU32(din, 0); /* 入力画像先頭アドレス */
  addw_keypoints = U64ToU32(din, 1); /* 出力特徴点配列先頭アドレス */

  /* ディスクリプタ読み出し */
  din      = ReadFifo(STP_CH0);
  addw_num = U64ToU32(din, 0); /* 出力特徴点の個数先頭アドレス */
  cols     = U64ToU16(din, 2); /* 入力画像サイズ（水平方向） */
  rows     = U64ToU16(din, 3); /* 入力画像サイズ（垂直方向） */

  FAST_all(addr, addw_keypoints, addw_num, rows, cols);

  CmdDinRestoreMode(STP_CH0);

#if defined(DRP6)
  CmdDoutInt();
#else
  CmdDoutInt(STP_CH0);
#endif

  return;
}

