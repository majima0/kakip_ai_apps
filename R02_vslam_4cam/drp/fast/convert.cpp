#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <experimental/filesystem>

#include "TBin_out.h"
#include "pgm_read_write.h"

int main(int argc, char *argv[]) {
  struct stat statBuf;

  const size_t BUF_SIZE = 100;
  size_t       FRANE_N;
  char         filename[BUF_SIZE];
  char         out_dir[BUF_SIZE];

  unsigned int image_size[8][2] = {
      {752, 480},
      {627, 400},
      {522, 333},
      {435, 278},
      {363, 231},
      {302, 193},
      {252, 161},
      {210, 134}};

  unsigned char src[image_size[0][0] * image_size[0][1]];
  memset(&src, 0x00, sizeof(src));

  if (argc != 3) {
    printf("Usage: ./convert.exe <frame number> <output directory path>\n");
    return 1;
  }

  FRANE_N = atoi(argv[1]);
  sprintf(out_dir, "%s", argv[2]);

  if (stat(out_dir, &statBuf) != 0) {
    if (!std::experimental::filesystem::create_directories(out_dir)) {
      printf("Failed to make %s directory\n", out_dir);
      return 1;
    }
  }

  for (size_t frame_id = 0; frame_id < FRANE_N; frame_id++) {
    for (size_t level = 0; level < 8; level++) {
      sprintf(filename, "image_pyramid/image_pyramid_%04lu_%lu.pgm", frame_id, level);
      if (pgm_read(src, image_size[level], filename)) {
        printf("failed to read %s\n", filename);
        return 1;
      }

      char benchname[100];
      sprintf(benchname, "fast_%04lu_%lu", frame_id, level);
      write_descriptor_file(out_dir, "16", benchname);
      write_initial_file(out_dir, src, image_size[level], 0x100000, 0x2000000, 0x3000000, benchname);
    }
  }

  return 0;
}
