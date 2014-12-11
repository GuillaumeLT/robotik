export LD_LIBRARY_PATH=/usr/local/Aria/lib
all:
	g++ -L/usr/local/Aria/lib -I'/usr/local/Aria/include/' RobotControl1.cpp -lAria -lpthread -ldl -lrt
