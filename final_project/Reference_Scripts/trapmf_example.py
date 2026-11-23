import numpy as np
import matplotlib.pyplot as plt

def trapmf(x,p):
	a,b,c,d = p
	x=np.asarray(x, float)
	y=np.zeros_like(x)
	if b>a:							# For rising case
		m=(x>a) & (x<b)
		y[m] =(x[m] - a) / (b-a)
	y[(x>=b)& (x<=c)] = 1.0			# Plateau case
	if d>c:							# For falling case
		m=(x>c) & (x<d)
		y[m] =(d- x[m]) / (d-c)
	return y

def plot(universe, mFunc, savename):
	plt.plot(universe, mFunc, lw=2.5, label=name)
	plt.savefig(savename)

def main(args):
    return 0

if __name__ == '__main__':
	DISTANCE = {
	"Triangle": [0,6,6,12],
	"Square": [6,6,12,12],
	"Down Slope": [0,0,3,6],
	}
	crisp_value = 5
	mf = trapmf(crisp_value, DISTANCE["Triangle"])
	
	print(f"Crisp value: {crisp_value} -> mf = {mf}")

	x=np.linspace(0,100,100)
	
	MF={}
	
	for name, params in DISTANCE.items():
		mf=trapmf(x, params)
		print(f"{name}: {mf}")
		MF[name]=mf
		
		
	plot(x,MF["Down Slope"], "DownSlope_MF.png")
	
	# Use Lambda for inline funcion definitions
