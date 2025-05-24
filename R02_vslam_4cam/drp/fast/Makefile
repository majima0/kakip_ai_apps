#
# $Id: Makefile,v 1.1 2019/01/28 04:31:50 hayashdi Exp $
#

APPLI		= fast

TB_IN		= $(APPLI)_TB_desc_in_csim.txt
TB_EXP_IN	= 
TB_IN_MEM	= $(APPLI)_TB_mem_in_csim.txt
TB_EXP_MEM	= $(APPLI)_TB_mem_out_exp.txt

CC		= gcc
FLAGS		= -O3 -Wall -W -g
INCLUDE		= -I../util
# FLAGS		= -Wall -W -Wl,--stack,10485760 -g
# please replace when Segmentation fault occured
SRCS		= reference.c

TARGET		= $(APPLI).exe
DEPEND		= $(patsubst %.c,%.d,$(SRCS))

all		: $(TARGET) $(DEPEND)

$(TARGET)	: $(SRCS)
	$(CC) $(FLAGS) $(INCLUDE) $(SRCS) -o $(TARGET) -lm

clean		:
	rm -rf $(TARGET) $(DEPEND) $(TB_IN) $(TB_EXP_IN) $(TB_IN_MEM) $(TB_EXP_MEM)

-include $(DEPEND)

$(DEPEND): $(SRCS)
	$(info generate $@)
	@$(CC) -MM $(FLAGS) $(INCLUDE) $(SRCS) | sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@
