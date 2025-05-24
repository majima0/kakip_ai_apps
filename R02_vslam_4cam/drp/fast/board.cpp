#include <stdint.h>
#include <stdio.h>

#include <fstream>

#include "AIPoC2N/lib/common.h"
#include "AIPoC2N/lib/r_dp2n.h"
#include "AIPoC2N/load_save_bin.h"
#include "io.h"
#include "measure_time.h"
#include "pgm_read_write.h"

#define MAX_NFEATURES 2000
#define MAX_KEYPOINTS (MAX_NFEATURES * 10)

#define PARAM_SIZE 16 /* size of fast_2849_0_TB_mem_in_csim_00000000.bin */

#define TRUE 1
#define FILE_NAME_LEN_MAX 255

#define DRP_INPUT_ADDRESS 0x68000000
#define DRP_OUTPUT_ADDRESS 0x68100000
#define DRP_OPTIONAL_ADDRESS 0x68200000
#define DRP_PARAM_ADDRESS 0x68600000
#define DRP_CONFIG_ADDRESS 0x68A00000

#define DRP_LOAD_DESC_ADDRESS 0x70000000

#define DRP_CONFIG_PATH "musketeer_fast/Embedded/fast_drp_out.bin"

struct Point2us {
  uint16_t x;
  uint16_t y;
};

struct KeyPoint {
  Point2us pt;
  int16_t  response;
  uint16_t padding;

  bool operator==(const KeyPoint &r) const {
    return (pt.x == r.pt.x) && (pt.y == r.pt.y) && (response == r.response);
  }
  bool operator!=(const KeyPoint &r) const {
    return !((pt.x == r.pt.x) && (pt.y == r.pt.y) && (response == r.response));
  }
};

measure_time_t mt_all;
measure_time_t mt_u2p_param;
measure_time_t mt_u2p_input;
measure_time_t mt_u2p_config;
measure_time_t mt_load;
measure_time_t mt_activate;
measure_time_t mt_start;
measure_time_t mt_polling;
measure_time_t mt_p2u_size;
measure_time_t mt_p2u;

std::vector<std::pair<measure_time_t *, std::string>> mt_list = {
    {&mt_u2p_param, "U2P parameter"},
    {&mt_u2p_input, "U2P input"},
    {&mt_u2p_config, "U2P configuration code"},
    {&mt_load, "R_DP2N_Load"},
    {&mt_activate, "R_DP2N_Activate"},
    {&mt_start, "R_DP2N_Start"},
    {&mt_polling, "busy wait"},
    {&mt_p2u_size, "P2U output keypoints size"},
    {&mt_p2u, "P2U output keypoints"}};

void verify(
    const KeyPoint output_keypoints[MAX_KEYPOINTS],
    const uint16_t output_keypoints_size,
    const uint16_t frame_id,
    const uint16_t level) {
  char          expect_name[FILE_NAME_LEN_MAX];
  KeyPoint      exp_keypoints[MAX_KEYPOINTS];
  std::ifstream ifs;

  sprintf(expect_name, "musketeer_fast/fast_%04hu_%hu_TB_mem_out_sim_02000000.bin", frame_id, level);
  ifs.open(std::string(expect_name), std::ios::binary);

  if (!ifs) {
    printf("Not found %s\n", expect_name);
    return;
  }
  ifs.read((char *)exp_keypoints, output_keypoints_size * 8);
  ifs.close();

  bool invalid = false;
  for (size_t i = 0; i < output_keypoints_size; i++) {
    const KeyPoint kp  = output_keypoints[i];
    const KeyPoint exp = exp_keypoints[i];
    if (kp != exp) {
      printf("not equals at [%lu] in %d level in %d frame. ", i, level, frame_id);

      printf("(%u, %u, %d) != (%u, %u, %d)\n", kp.pt.x, kp.pt.y, kp.response, exp.pt.x, exp.pt.y, exp.response);
      invalid = true;
      break;
    }
    if (invalid) break;
  }
}

int main(int argc, char **argv) {
  FILE    *fp;
  uint32_t i;
  KeyPoint output_keypoints[MAX_KEYPOINTS];
  uint16_t output_keypoints_size;
  uint32_t parameter[4];
  uint32_t config_size;
  char     input_name[FILE_NAME_LEN_MAX];
  char     output_name[FILE_NAME_LEN_MAX];
  size_t   measure_times = 1;

  R_DP2N_Status stat;
  int8_t        ret;
  uint8_t       drp_dev_num;
  uint16_t      frame_id, level;
  uint16_t      rows, cols;

  if (argc < 6 || 7 < argc) {
    printf("Usage : %s drp_dev_num[0 to 4] <frame_id> <level> <column size> <row size> [measure times]\n", argv[0]);
    return 1;
  }

  sscanf(argv[1], "%hhu", &drp_dev_num);
  sscanf(argv[2], "%hu", &frame_id);
  sscanf(argv[3], "%hu", &level);
  sscanf(argv[4], "%hu", &cols);
  sscanf(argv[5], "%hu", &rows);
  if (argc == 7) sscanf(argv[6], "%lu", &measure_times);

  sprintf(input_name, "musketeer_fast/fast_%04hu_%hu_TB_mem_in_csim_00100000.bin", frame_id, level);

  if (drp_dev_num >= 5) {
    printf("Usage : %s drp_dev_num[0 to 4] <frame_id> <level> <column size> <row size> [measure times]\n", argv[0]);
    return 1;
  }

  parameter[0] = DRP_INPUT_ADDRESS;
  parameter[1] = DRP_OUTPUT_ADDRESS;    // consider w_addr_keypoints
  parameter[2] = DRP_OPTIONAL_ADDRESS;  // consider w_addr_num
  parameter[3] = cols + (rows << 16);
  config_size  = get_configuration_size(DRP_CONFIG_PATH);

  ret = R_DP2N_Initialize(drp_dev_num);
  if (ret) {
    printf("R_DP2N_Initialize error(%d)\n", ret);
    return 1;
  }

  for (size_t times = 0; times < measure_times; times++) {
    MT_START(mt_all);

    MT_START(mt_u2p_param);
    R_DP2N_MemcpyU2P((void *)DRP_PARAM_ADDRESS, parameter, PARAM_SIZE);
    MT_FINISH(mt_u2p_param);

    MT_START(mt_u2p_input);
    load_bin(input_name, DRP_INPUT_ADDRESS, -1);
    MT_FINISH(mt_u2p_input);

    MT_START(mt_u2p_config);
    load_bin(DRP_CONFIG_PATH, DRP_CONFIG_ADDRESS, -1);
    MT_FINISH(mt_u2p_config);

    MT_START(mt_load);
    ret = R_DP2N_Load(drp_dev_num, DRP_MODE_DRP_ONLY, (uint32_t *)DRP_CONFIG_ADDRESS, config_size, 1, DRP_LOAD_DESC_ADDRESS);
    if (ret) {
      printf("R_DP2N_Load error(%d)\n", ret);
      return 1;
    }
    MT_FINISH(mt_load);

    MT_START(mt_activate);
    ret = R_DP2N_Activate(drp_dev_num, DRP_AI_FREQ_DIV_3, AIMAC_FREQ_800MHz);
    if (ret) {
      printf("R_DP2N_Activate error(%d)\n", ret);
      return 1;
    }
    MT_FINISH(mt_activate);

    MT_START(mt_start);
    ret = R_DP2N_Start(drp_dev_num, (uint32_t *)DRP_PARAM_ADDRESS, PARAM_SIZE, NULL, 0, NULL /* no callback */, DRP_LOAD_DESC_ADDRESS);
    if (ret) {
      printf("R_DP2N_Start error(%d)\n", ret);
      return 1;
    }
    MT_FINISH(mt_start);

    // busy wait
    MT_START(mt_polling);
    while (TRUE) {
      R_DP2N_GetStatus(drp_dev_num, &stat);
      if (stat.stpIntNum == 1) break;
    }
    MT_FINISH(mt_polling);

    MT_START(mt_p2u_size);
    R_DP2N_MemcpyP2U(&output_keypoints_size, (void *)DRP_OPTIONAL_ADDRESS, 2);
    MT_FINISH(mt_p2u_size);

    MT_START(mt_p2u);
    R_DP2N_MemcpyP2U(output_keypoints, (void *)DRP_OUTPUT_ADDRESS, output_keypoints_size * 8);
    MT_FINISH(mt_p2u);

    R_DP2N_Reset(drp_dev_num);

    MT_FINISH(mt_all);
  }

  sprintf(output_name, "board_result.csv");

  if ((fp = fopen(output_name, "w")) == NULL) {
    printf("Can't create file.\n");
    return 1;
  }

  fprintf(fp, "x,y,angle,class_id, octave,response,size\n");
  for (i = 0; i < output_keypoints_size; i++) {
    output_keypoints[i].response = (int16_t)(output_keypoints[i].response & 0xFF);
    KeyPoint kp                  = output_keypoints[i];
    fprintf(fp, "%u,%u,%.0f,%d,%u,%d,%u\n", kp.pt.x, kp.pt.y, -1.0f, -1, 0, kp.response, 7);
  }

  fclose(fp);

  measure_time_result(mt_list, mt_all, measure_times);

  verify(output_keypoints, output_keypoints_size, frame_id, level);
}
