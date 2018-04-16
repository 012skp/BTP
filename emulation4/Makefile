main: main.cc controller.o link.o switch.o
	g++ -std=c++11 main.cc -lpthread
	./a.out

plotload:
	gnuplot -p -e "load \"gnuplot_script1\""

plotTP:
	gnuplot -p -e "load \"gnuplot_script2\""

controller.o: controller.cc
	g++ -std=c++11 -c controller.cc -lpthread

link.o: link.cc
	g++ -std=c++11 -c link.cc -lpthread

switch.o: switch.cc
	g++ -std=c++11 -c switch.cc -lpthread

clean:
	rm -rf *.o a.out throughput pkt_drop c0_load c1_load
