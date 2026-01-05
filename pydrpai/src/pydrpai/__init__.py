from pydrpai._core import (
    init_drpai,
    PreRuntime,
    MeraDrpRuntimeWrapper,
    buffer_alloc_dmabuf,
    memcpy,
    buffer_flush_dmabuf,
    s_preproc_param_t,
    get_result,
)

def init_drpai1(fd) -> int:
    return init_drpai(fd)
