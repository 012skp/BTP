main: main.cc controller.o link.o switch.o
	g++ -std=c++11 main.cc -lpthread
	./a.out
plot:
	gnuplot -p -e "load \"gnuplot_script1\""
	#gnuplot -p -e "load \"gnuplot_script2\""
controller.o: controller.cc
	g++ -std=c++11 -c controller.cc -lpthread
link.o: link.cc
	g++ -std=c++11 -c link.cc -lpthread
switch.o: switch.cc
	g++ -std=c++11 -c switch.cc -lpthread
