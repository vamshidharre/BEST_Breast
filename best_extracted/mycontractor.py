from myinterval import *

def cadd(z,x,y):
	return z&x+y, x&z-y, y&z-x	

def cminus(z,x,y):  # z=x-y
	return z&x-y, x&z+y, y&x-z


def cmul(z,x,y):
	return z&x*y, x&z/y, y&z/x	
	
def cexp(y,x):
	return y&exp(x), x&log(y)
	
def csqr(y,x):
	x1=x&Interval(-oo,0); 
	x2=x&Interval(0,oo); 	
	y=(y&sqr(x1))|(y&sqr(x2));
	if y.is_empty(): 
		return y,y
	else:
		x1=x1&(-1*sqrt(y));
		x2=x2&(sqrt(y));
		return y,x1|x2;
	
def cdistance(d,x1,y1,x2,y2):
	X = Interval(-oo,oo)
	Y = Interval(-oo,oo)
	X2 = Interval(0,oo)
	Y2 = Interval(0,oo)
	D2 = Interval(0,oo)
	X,x2,x1 = cminus(X,x2,x1)
	Y,y2,y1 = cminus(Y,y2,y1)
	X2,X = csqr(X2,X)
	Y2,Y = csqr(Y2,Y)
	D2,X2,Y2 = cadd(D2,X2,Y2)
	D2,d = csqr(D2,d)
	D2,X2,Y2 = cadd(D2,X2,Y2)
	X2,X = csqr(X2,X)
	Y2,Y = csqr(Y2,Y)
	X,x2,x1 = cminus(X,x2,x1)
	Y,y2,y1 = cminus(Y,y2,y1)
	return d,x1,y1,x2,y2

cdist = cdistance
	

if __name__ == '__main__':

	# a = Interval(-5,3)
	# b = Interval(4,10)
	# c = Interval(12,100)
	# print("a = ",a,"b = ",b,"c = ",c)
	# c,a,b=cadd(c,a,b)
	# print("a = ",a,"b = ",b,"c = ",c)
	
	a = Interval(-1,2)
	b = Interval(4,10)
	print("a = ",a,"b = ",b)
	b,a=csqr(b,a)
	print("a = ",a,"b = ",b)
	
	
	
	
	

