import matplotlib.pyplot as plt
import numpy as np
from math import *
def bernstein_polynomials(nu,n,x):
	if nu<0 or nu>n:
		return 0
	binomial_coeff = factorial(n)/factorial(nu)/factorial(n-nu)
	return binomial_coeff*x**nu*(1-x)**(n-nu);
	
def bazier_curve(Points,weight,T,dt):
	n = len(Points)
	P=[]
	P_prime=[]	
	Points = np.array(Points)
	weight = np.array(weight)
	for j in range(int(T/dt)):
		t= j*dt;
		a=0;
		a_prime=0;
		b_prime=0;		
		b=0;		
		for i in range(0,n):
			B = bernstein_polynomials(i,n,t)
			B_prime = n*(bernstein_polynomials(i-1,n-1,t)-bernstein_polynomials(i,n-1,t))
			a_prime = a_prime+(B_prime*Points[i]*weight[i])
			a_prime = b_prime+(B_prime*weight[i])
			a = a+(B*Points[i]*weight[i])
			b = b+(B*weight[i])
		P.append(a/b)
		P_prime.append((a_prime-b_prime*(a/b))/b)	
	return np.array(P),np.array(P_prime)

Points=[[0,0],[0.5,1],[1,0]]

w = [1,1,1]
T = 1
dt = 0.001

P,P_prime= bazier_curve(Points,w,T,dt)
print(P)
ax1 = plt.subplot(2, 1, 1)
ax2 = plt.subplot(2, 1, 2)

ax1.plot(P[:,0],P[:,1])
ax1.set_aspect('equal')
ax2.plot(P_prime[:,0],P_prime[:,1])
ax2.set_aspect('equal')
plt.show()

'''	
ax1 = plt.subplot(1, 1, 1)
dt = 0.001;
x = [0 ,0 ,1,1]
dx= [0 ,0 ,1,0]
y = [0 ,1 ,1,0]
dy= [0 ,1 ,0,0]

ax1.plot(x,y)
P=[[0,0],[0,1],[1,1],[1,0]]

traj,dot_traj,ddot_traj =bazier_curve(P,1,dt)
print(traj.shape)
ax1.plot(traj[:,0],traj[:,1])
ax1.set_aspect('equal')

plt.show()
'''




