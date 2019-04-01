CROSS		=	
PYTHON		= python
CC			= $(CROSS)gcc
CXX			= $(CROSS)g++
AR			= $(CROSS)ar
CFLAGS		= -Wall -fPIC -g
CXXFLAGS	= -Wall -fPIC -g
LDSHFLAGS	= -rdynamic -shared 
ARFLAGS		= rcv
CODE_STYLE	= astyle --align-pointer=name --align-reference=name --suffix=none --break-blocks --pad-oper --pad-header --break-blocks --keep-one-line-blocks --indent-switches --indent=tab=4

SOURCES=$(wildcard src/*.cpp)
HEADERS=$(wildcard src/*.h)
OBJECTS=$(SOURCES:.cpp=.o)
TARGETS = libserial.a libserial.so

.PHONY:all clean example test install help style
.SILENT: clean

all:$(TARGETS)

clean:
	find . -name "*.o" | xargs rm -f 
	$(RM) *.o *.so *~ a.out depend.d $(TARGETS) build -rf

style:
	@find -regex '.*/.*\.\(c\|cpp\|h\)$$' | xargs $(CODE_STYLE)

install:
	$(PYTHON) setup.py install

libserial.a:$(OBJECTS)
	$(AR) $(ARFLAGS) $@ $^

libserial.so:$(OBJECTS)
	$(CC) $(LDSHFLAGS) -o $@ $^

depend.d:$(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) -MM $^ > $@

-include depend.d
