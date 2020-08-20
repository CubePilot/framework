MODULE_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
INCDIR += $(MODULE_DIR)/eigen
UDEFS += -DEIGEN_NO_MALLOC -DEIGEN_NO_AUTOMATIC_RESIZING -DEIGEN_UNROLLING_LIMIT=1000 -DEIGEN_NO_DEBUG -DEIGEN_MALLOC_ALREADY_ALIGNED=true -DCHPRINTF_USE_FLOAT=TRUE
