#include<tclcl.h>

extern EmbeddedTcl PackerUwmsunTclCode;

extern "C" int Packeruwmsun_Init() {
	PackerUwmsunTclCode.load();
	return 0;
}
