from mycontractor import *
from roblib import *


_x,_y=[66,-44,-9,-30,12,-20,0,-50],[-70,55,30,-15,70,0,10,-50]
N=len(_x)
_d=[[0. for i in range(N)] for j in range(N)]
D=[[Interval(-oo,oo) for i in range(N)] for j in range(N)]
X=[Interval(-oo,oo) for i in range(N)]
Y=[Interval(-oo,oo) for i in range(N)]
for i in range(0,N):
	for j in range(0,N):
		if i!=j:
			_d[i][j]=int(round(sqrt((_x[j]-_x[i])**2+(_y[j]-_y[i])**2)))
			D[i][j]=Interval(_d[i][j]-0.5,_d[i][j]+0.5)
		else: D[i][j]=Interval(0,0)


ax=init_figure(-100,100,-100,100)


for i in range(0,3):
	X[i]=Interval(_x[i]-3,_x[i]+3)
	Y[i]=Interval(_y[i]-3,_y[i]+3)
	draw_box_border(X[i].lb,X[i].ub,Y[i].lb,Y[i].ub,'blue',1)

print("_d=",_d)
print("Initial X=",X)
print("Initial Y=",Y)

for k in range(20):
	for i in range(N):
		for j in range(N):
			if i != j:
				D[i][j], X[i], Y[i], X[j], Y[j] = cdistance(D[i][j], X[i], Y[i], X[j], Y[j])

for i in range(0,N):
	draw_box_border(X[i].lb,X[i].ub,Y[i].lb,Y[i].ub,'magenta',1)

for i in range(0,N):
	draw_box_border(_x[i]-0.2,_x[i]+0.2,_y[i]-0.2,_y[i]+0.2,'red',2)

print("\nFinal localized boxes:")
for i in range(N):
	print(f"Robot {i}: true=({_x[i]}, {_y[i]}), X={X[i]}, Y={Y[i]}, width=({X[i].width():.2f}, {Y[i].width():.2f})")

pause(10)


