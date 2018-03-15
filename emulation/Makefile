main: main.cc controller.o link.o
	g++ -std=c++11 main.cc -lpthread
	./a.out
	gnuplot -p -e "load \"gnuplot_script\""
controller.o: controller.cc
	g++ -std=c++11 -c controller.cc -lpthread
link.o: link.cc
	g++ -std=c++11 -c link.cc -lpthread
