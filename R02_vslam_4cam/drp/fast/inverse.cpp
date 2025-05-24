#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <experimental/filesystem>

#include "TBin_out.h"

int main(int argc, char *argv[]) {
  struct stat statBuf;

  const size_t BUF_SIZE = 100;
  size_t       FRANE_N;
  char         in_dir[BUF_SIZE];
  char         out_dir[BUF_SIZE];
  char         filename[BUF_SIZE];

  if (argc != 4) {
    printf("Usage: ./inverse.exe <frame number> <input directory path> <output directory path>\n");
    return 1;
  }

  FRANE_N = atoi(argv[1]);
  sprintf(in_dir, "%s", argv[2]);
  sprintf(out_dir, "%s", argv[3]);

  if (stat(in_dir, &statBuf) != 0) {
    printf("Not found %s\n", in_dir);
    return 1;
  }
  if (stat(out_dir, &statBuf) != 0) {
    if (!std::experimental::filesystem::create_directories(out_dir)) {
      printf("Failed to make %s directory\n", out_dir);
      return 1;
    }
  }

  for (size_t frame_id = 0; frame_id < FRANE_N; frame_id++) {
    for (size_t level = 0; level < 8; level++) {
      sprintf(filename, "fast_%04lu_%lu", frame_id, level);

      if (!TB_to_pgm(in_dir, out_dir, filename, 0x2000000, 0x3000000)) return 1;
    }
  }

  return 0;
}
