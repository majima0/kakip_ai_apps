#ifndef DRP_FAST_TBIN_OUT_H
#define DRP_FAST_TBIN_OUT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "TBin_out_impl.h"
#include "datatype.h"

#define FILE_NAME_LEN_MAX 255
#define FIXED_POINT16 65536 /* 2^16 */

/*---------------------------------------------------------------------------
 テストベンチ入力ファイル生成
---------------------------------------------------------------------------*/

void desc_space(
    FILE         *fp,
    unsigned int *image_size,       /* 画像サイズ配列ポインタ */
    unsigned int  r_addr,           /* SDRAM読み込み領域先頭アドレス */
    unsigned int  w_addr_keypoints, /* SDRAM書き込み領域先頭アドレス */
    unsigned int  w_addr_num,       /* SDRAM書き込み領域先頭アドレス */
    char         *pragma) {
  fprintf(fp, "@00000000\n");
  /* SDRAM読み込み領域先頭アドレス */
  fprintf(fp, "%02x%s\n", (unsigned char)((r_addr)&0xFF), pragma);
  fprintf(fp, "%02x%s\n", (unsigned char)((r_addr >> 8) & 0xFF), pragma);
  fprintf(fp, "%02x%s\n", (unsigned char)((r_addr >> 16) & 0xFF), pragma);
  fprintf(fp, "%02x%s\n", (unsigned char)((r_addr >> 24) & 0xFF), pragma);
  /* SDRAM書き込み領域先頭アドレス */
  fprintf(fp, "%02x%s\n", (unsigned char)((w_addr_keypoints)&0xFF), pragma);
  fprintf(fp, "%02x%s\n", (unsigned char)((w_addr_keypoints >> 8) & 0xFF), pragma);
  fprintf(fp, "%02x%s\n", (unsigned char)((w_addr_keypoints >> 16) & 0xFF), pragma);
  fprintf(fp, "%02x%s\n", (unsigned char)((w_addr_keypoints >> 24) & 0xFF), pragma);
  /* SDRAM書き込み領域先頭アドレス */
  fprintf(fp, "%02x%s\n", (unsigned char)((w_addr_num)&0xFF), pragma);
  fprintf(fp, "%02x%s\n", (unsigned char)((w_addr_num >> 8) & 0xFF), pragma);
  fprintf(fp, "%02x%s\n", (unsigned char)((w_addr_num >> 16) & 0xFF), pragma);
  fprintf(fp, "%02x%s\n", (unsigned char)((w_addr_num >> 24) & 0xFF), pragma);
  /* 画像横方向サイズ */
  fprintf(fp, "%02x\n", (unsigned char)((image_size[0]) & 0xFF));
  fprintf(fp, "%02x\n", (unsigned char)((image_size[0] >> 8) & 0xFF));
  /* 画像縦方向サイズ */
  fprintf(fp, "%02x\n", (unsigned char)((image_size[1]) & 0xFF));
  fprintf(fp, "%02x\n", (unsigned char)((image_size[1] >> 8) & 0xFF));
}

void input_space(
    FILE          *fp,
    unsigned char *inbuf,      /* 入力画像格納先頭ポインタ */
    unsigned int  *image_size, /* 画像サイズ配列ポインタ */
    unsigned int   r_addr      /* SDRAM読み込み領域先頭アドレス */
) {
  size_t i;
  /* 入力画像データ */
  fprintf(fp, "@%08x\n", r_addr);
  for (i = 0; i < image_size[0] * image_size[1]; i++) {
    fprintf(fp, "%02x\n", inbuf[i]);
  }
}

void output_space_init(
    FILE         *fp,
    unsigned int *image_size,       /* 画像サイズ配列ポインタ */
    unsigned int  w_addr_keypoints, /* SDRAM書き込み領域先頭アドレス */
    unsigned int  w_addr_num        /* SDRAM書き込み領域先頭アドレス */
) {
  size_t i;
  /* 出力画像データ */
  fprintf(fp, "@%08x\n", w_addr_keypoints);
  for (i = 0; i < image_size[0] * image_size[1]; i++) {
    fprintf(fp, "%02x\n", 0);
  }
  fprintf(fp, "@%08x\n", w_addr_num);
  fprintf(fp, "00\n00\n00\n00\n00\n00\n00\n00\n");
}

void output_space_exp(
    FILE         *fp,
    KeyPoint     *outbuf,           /* 出力特徴点配列格納先頭ポインタ */
    unsigned int *image_size,       /* 画像サイズ配列ポインタ */
    unsigned int  keypts_size,      /* 特徴点数 */
    unsigned int  w_addr_keypoints, /* SDRAM書き込み領域先頭アドレス */
    unsigned int  w_addr_num        /* SDRAM書き込み領域先頭アドレス */
) {
  size_t i;
  /* 出力画像データ */
  fprintf(fp, "@%08x\n", w_addr_keypoints);
  for (i = 0; i < keypts_size; i++) {
    fprintf(fp, "%02x\n", (unsigned char)(((unsigned short)outbuf[i].pt.x) & 0xFF));
    fprintf(fp, "%02x\n", (unsigned char)(((unsigned short)outbuf[i].pt.x) >> 8 & 0xFF));
    fprintf(fp, "%02x\n", (unsigned char)(((unsigned short)outbuf[i].pt.y) & 0xFF));
    fprintf(fp, "%02x\n", (unsigned char)(((unsigned short)outbuf[i].pt.y) >> 8 & 0xFF));
    fprintf(fp, "%02x\n", (unsigned char)outbuf[i].response);
    /* ダミーデータ */
    fprintf(fp, "00\n00\n00\n");
  }

  /* ダミーデータ */
  for (; i < image_size[0] * image_size[1] / 8; i++) {
    fprintf(fp, "00\n00\n00\n00\n00\n00\n00\n00\n");
  }
  for (i = 0; i < (image_size[0] * image_size[1]) % 8; i++) {
    fprintf(fp, "00\n");
  }

  /* 出力特徴点の個数 */
  fprintf(fp, "@%08x\n", w_addr_num);
  fprintf(fp, "%02x\n", (unsigned char)((keypts_size)&0xFF));
  fprintf(fp, "%02x\n", (unsigned char)((keypts_size >> 8) & 0xFF));
  fprintf(fp, "%02x\n", (unsigned char)((keypts_size >> 16) & 0xFF));
  fprintf(fp, "%02x\n", (unsigned char)((keypts_size >> 24) & 0xFF));
  /* ダミーデータ */
  fprintf(fp, "00\n00\n00\n00\n");
}

void write_initial_file(
    const char    *out_dir,          /* 出力先 */
    unsigned char *inbuf,            /* 入力画像格納先頭ポインタ */
    unsigned int  *image_size,       /* 画像サイズ配列ポインタ */
    unsigned int   r_addr,           /* SDRAM読み込み領域先頭アドレス */
    unsigned int   w_addr_keypoints, /* SDRAM書き込み領域先頭アドレス */
    unsigned int   w_addr_num,       /* SDRAM書き込み領域先頭アドレス */
    char          *TB_file_Top       /* ファイル名先頭文字列 */
) {
  char  filename[FILE_NAME_LEN_MAX];
  FILE *fp;

  sprintf(filename, "%s/%s_TB_mem_in_csim.txt", out_dir, TB_file_Top);
  if ((fp = fopen(filename, "w")) == NULL) {
    printf("Can't create file.\n");
    exit(1);
  }

  desc_space(fp, image_size, r_addr, w_addr_keypoints, w_addr_num, " // EXTMEM_ADR");
  input_space(fp, inbuf, image_size, r_addr);
  output_space_init(fp, image_size, w_addr_keypoints, w_addr_num);

  fclose(fp);
}

void write_expected_file(
    const char    *out_dir,          /* 出力先 */
    unsigned char *inbuf,            /* 入力画像格納先頭ポインタ */
    KeyPoint      *outbuf,           /* 出力特徴点配列格納先頭ポインタ */
    unsigned int  *image_size,       /* 画像サイズ配列ポインタ */
    unsigned int   keypts_size,      /* 特徴点数 */
    unsigned int   r_addr,           /* SDRAM読み込み領域先頭アドレス */
    unsigned int   w_addr_keypoints, /* SDRAM書き込み領域先頭アドレス */
    unsigned int   w_addr_num,       /* SDRAM書き込み領域先頭アドレス */
    char          *TB_file_Top       /* ファイル名先頭文字列 */
) {
  char  filename[FILE_NAME_LEN_MAX];
  FILE *fp;

  sprintf(filename, "%s/%s_TB_mem_out_exp.txt", out_dir, TB_file_Top);
  if ((fp = fopen(filename, "w")) == NULL) {
    printf("Can't create file.\n");
    exit(1);
  }

  desc_space(fp, image_size, r_addr, w_addr_keypoints, w_addr_num, "");
  input_space(fp, inbuf, image_size, r_addr);
  output_space_exp(fp, outbuf, image_size, keypts_size, w_addr_keypoints, w_addr_num);

  fclose(fp);
}

void TBin_out(
    const char    *out_dir,          /* 出力先 */
    unsigned char *inbuf,            /* 入力画像格納先頭ポインタ */
    KeyPoint      *outbuf,           /* 出力特徴点配列格納先頭ポインタ */
    unsigned int  *image_size,       /* 画像サイズ配列ポインタ */
    unsigned int   keypts_size,      /* 特徴点数 */
    unsigned int   r_addr,           /* SDRAM読み込み領域先頭アドレス */
    unsigned int   w_addr_keypoints, /* SDRAM書き込み領域先頭アドレス */
    unsigned int   w_addr_num,       /* SDRAM書き込み領域先頭アドレス */
    char          *TB_file_Top       /* ファイル名先頭文字列 */
) {
  write_descriptor_file(out_dir, "16", TB_file_Top);
  write_initial_file(out_dir, inbuf, image_size, r_addr, w_addr_keypoints, w_addr_num, TB_file_Top);
  write_expected_file(out_dir, inbuf, outbuf, image_size, keypts_size, r_addr, w_addr_keypoints, w_addr_num, TB_file_Top);

  return;
}

unsigned int TB_to_pgm(
    const char        *in_dir,
    const char        *out_dir,
    const char        *filename_prefix,
    const unsigned int w_addr_keypoints,
    const unsigned int w_addr_num) {
  char         filename[FILE_NAME_LEN_MAX];
  FILE        *fp;
  FILE        *dst;
  size_t       i;
  char         csvname[FILE_NAME_LEN_MAX];
  char         address[100];
  char         line[100]   = {'\0'};
  unsigned int keypts_size = 0;

  sprintf(filename, "%s/%s_TB_mem_out_csim.txt", in_dir, filename_prefix);
  sprintf(csvname, "%s/%s.csv", out_dir, filename_prefix);

  sprintf(address, "@%08x\n", w_addr_num);

  if ((fp = fopen(filename, "rb")) == NULL) {
    printf("Can't read file.\n");
    return FALSE;
  }

  if ((dst = fopen(csvname, "w")) == NULL) {
    printf("Can't read file.\n");
    return FALSE;
  }

  /* search starting address */
  while (fgets(line, 100, fp) != NULL) {
    if (strcmp(line, address) == 0) break;
  }

  for (i = 0; i < 4; i++) {
    if (fgets(line, 100, fp) == NULL) return FALSE;
    keypts_size += strtol(line, NULL, 16) << (8 * i);
  }

  fclose(fp);

  if ((fp = fopen(filename, "rb")) == NULL) {
    printf("Can't read file.\n");
    return FALSE;
  }

  sprintf(address, "@%08x\n", w_addr_keypoints);

  /* search starting address */
  while (fgets(line, 100, fp) != NULL) {
    if (strcmp(line, address) == 0) break;
  }

  fprintf(dst, "x,y,angle,class_id, octave,response,size\n");
  for (i = 0; i < keypts_size; i++) {
    if (fgets(line, 100, fp) == NULL) return FALSE;
    unsigned char x_lo = strtol(line, NULL, 16);
    if (fgets(line, 100, fp) == NULL) return FALSE;
    unsigned char x_hi = strtol(line, NULL, 16);
    if (fgets(line, 100, fp) == NULL) return FALSE;
    unsigned char y_lo = strtol(line, NULL, 16);
    if (fgets(line, 100, fp) == NULL) return FALSE;
    unsigned char y_hi = strtol(line, NULL, 16);
    if (fgets(line, 100, fp) == NULL) return FALSE;
    unsigned char res_lo = strtol(line, NULL, 16);
    if (fgets(line, 100, fp) == NULL) return FALSE;
    unsigned char res_hi = strtol(line, NULL, 16);
    /* read empty value */
    if (fgets(line, 100, fp) == NULL) return FALSE;
    if (fgets(line, 100, fp) == NULL) return FALSE;

    unsigned short x        = x_lo + (x_hi << 8);
    unsigned short y        = y_lo + (y_hi << 8);
    unsigned short response = res_lo + (res_hi << 8);

    fprintf(dst, "%u,%u,-1,-1,0,%u,7\n", x, y, response);
  }

  fclose(fp);
  fclose(dst);

  return TRUE;
}

#endif  // DRP_FAST_TBIN_OUT_H
