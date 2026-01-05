#ifndef UTILS_H_
#define UTILS_H_

/*DRP-AI TVM[*1] Runtime*/
#include "MeraDrpRuntimeWrapper.h"
/*Pre-processing Runtime Header*/
#include "PreRuntime.h"
/*Definition of Macros & other variables*/
#include "define.h"
/*box drawing*/
#include "box.h"
/*dmabuf for Pre-processing Runtime input data*/
#include "dmabuf.h"

uint64_t init_drpai(int drpai_fd);

std::vector<detection> get_result(MeraDrpRuntimeWrapper runtime);

#endif // UTILS_H_
