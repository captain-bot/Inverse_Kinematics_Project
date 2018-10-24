sol_baxik: my_baxter_IK.o IKfunctions.o
	g++ -I/home/nobug-ros/Documents/armadillo/include my_baxter_IK.o IKfunctions.o -o sol_baxik -llapack -lblas

my_baxter_IK.o: my_baxter_IK.cpp IKfunctions.h
	g++ -std=c++11 -c -I/home/nobug-ros/Documents/armadillo/include -O2 my_baxter_IK.cpp -llapack -lblas

IKfunctions.o: IKfunctions.cpp IKfunctions.h
	g++ -std=c++11 -c -I/home/nobug-ros/Documents/armadillo/include -O2 IKfunctions.cpp -llapack -lblas

clean:
	rm *.o sol_baxik

