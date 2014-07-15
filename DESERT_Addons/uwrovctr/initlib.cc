#include<tclcl.h>

extern EmbeddedTcl UwrovctrTclCode;

extern "C" int Uwrovctr_Init() {
	UwrovctrTclCode.load();
	return 0;
}
