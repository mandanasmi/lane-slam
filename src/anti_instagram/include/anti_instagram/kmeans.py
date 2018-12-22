from collections import Counter
from sklearn import linear_model
from sklearn.cluster import KMeans
import cv2
import numpy as np
import sys
import time

CENTERS2 = np.array([[60, 60, 60],[60, 60, 240], [50, 240, 240], [240, 240, 240]]);
CENTERS = np.array([[60, 60, 60], [50, 240, 240], [240, 240, 240]])
# in HSV: [0,0,60], [127.5000  233.7500  240.0000],[0,0,240]


def getimgdatapts(cv2img):
    x, y, p = cv2img.shape
    cv2_tpose = cv2img.transpose()
    cv2_arr_tpose = np.reshape(cv2_tpose,[p,x*y])
    npdata = np.transpose(cv2_arr_tpose);
    return npdata

#priors
def runKMeans(cv_img, num_colors, init):
    
	imgdata = getimgdatapts(cv_img[-100:,:,:]) # FIX ME: arbitrary cut off
	kmc = KMeans(n_clusters=num_colors, max_iter=25, n_init=10, init=init)
	t1 = time.time();
	kmc.fit_predict(imgdata)
	t2 = time.time();
	print("fit time: %f"%(t2-t1))
	trained_centers = kmc.cluster_centers_
	# print trained_centers
	labels = kmc.labels_
	labelcount = Counter()
	t1 = time.time();
	# IPython.embed()
	# for pixel in labels:
	# 	labelcount[pixel] += 1
	for i in np.arange(num_colors):
		labelcount[i]=np.sum(labels==i)

	t2 = time.time();
	print("counting labels time: %f"%(t2-t1))
	# print labelcount
	# IPython.embed()
	score=kmc.score(imgdata)
	return trained_centers, labelcount,score


def identifyColors(trained, true):
	# print trained
	# print np.size(trained)
	numcolors, _ = np.shape(trained)
	# print numcolors
	matching = np.zeros((numcolors, numcolors))
	# print matching
	for i, color in enumerate(trained):
		# print color
		for j, truecolor in enumerate(true):
			matching[i][j] = np.linalg.norm(color - truecolor)

	# print matching
	colormap= {}
	for i, color in enumerate(matching):
		colormap[i] = np.argmin(color)
		# colormap[2] = 1
		# print colormap
	colormap = checkMapping(colormap)
	return colormap

def checkMapping(mymap):
	maplist = []
	clearmap = {}
	for color, mapping in mymap.iteritems():
		if mapping not in maplist:
			clearmap[color] = mapping
			maplist += mapping
	# print clearmap
	return clearmap

def getparameters2(mapping, trained, weights, true):
	redX = np.zeros((3, 1))
	redY = np.zeros((3, 1))
	greenX = np.zeros((3, 1))
	greenY = np.zeros((3, 1))
	blueX = np.zeros((3, 1))
	blueY = np.zeros((3, 1))
	prior_redX = np.zeros((3, 1))
	prior_redY = np.zeros((3, 1))
	prior_greenX = np.zeros((3, 1))
	prior_greenY = np.zeros((3, 1))
	prior_blueX = np.zeros((3, 1))
	prior_blueY = np.zeros((3, 1))
	# print type(redY), redX
	# print trained, true
	prior_trained=np.array([[255, 0, 0],[0, 255, 0],[0, 0, 255]])
	prior_true=np.array([[255, 0, 0],[0, 255, 0],[0, 0, 255]])
	diagonal_prior_weight=300 # the coefficients along the diagonal should be close to each other - i.e close to "white" light
	a_prior_weight=0.2 # a should be close to 1
	INFEASIBILITY_PENALTY=1000000
	
	min_fitting_cost=np.inf
	t1=time.time()

	# perms=itertools.permutations([0,1,2])
	perms=[[0,1,2]]
	for perm in perms:
		redY[:,0]=true[:,0]# BGR..
		greenY[:,0]=true[:,1]
		blueY[:,0]=true[:,2]
		prior_redY[:,0]=prior_true[:,0]# BGR..
		prior_greenY[:,0]=prior_true[:,1]
		prior_blueY[:,0]=prior_true[:,2]
		redX[:,0]=trained[:,perm[0]]
		greenX[:,0]=trained[:,perm[1]]
		blueX[:,0]=trained[:,perm[2]]
		

		prior_redX[:,0]=prior_trained[:,perm[0]]
		prior_greenX[:,0]=prior_trained[:,perm[1]]
		prior_blueX[:,0]=prior_trained[:,perm[2]]
		redY2=np.concatenate((redY,prior_redY),axis=0)
		greenY2=np.concatenate((greenY,prior_greenY),axis=0)
		blueY2=np.concatenate((blueY,prior_blueY),axis=0)
		redX2=np.concatenate((redX,prior_redX),axis=0)
		greenX2=np.concatenate((greenX,prior_greenX),axis=0)
		blueX2=np.concatenate((blueX,prior_blueX),axis=0)
		sumweights=np.double(np.sum([weights[0],weights[1],weights[2]]))
		weightsMTX=np.double(np.diagflat([weights[0],weights[1],weights[2]]))
		weightsMTX=weightsMTX/sumweights
		# color fitting terms
		A1=np.concatenate((np.dot(weightsMTX,np.concatenate((redX,np.ones(np.shape(redX)),redX*0,redX*0,redX*0,redX*0),1)),np.dot(weightsMTX,np.concatenate((greenX*0,greenX*0,greenX,np.ones(np.shape(greenX)),greenX*0,greenX*0),1)),np.dot(weightsMTX,np.concatenate((blueX*0,blueX*0,blueX*0,blueX*0,blueX,np.ones(np.shape(blueX))),1))),0)
		b1=np.concatenate((np.dot(weightsMTX,redY),np.dot(weightsMTX,greenY),np.dot(weightsMTX,blueY)),0)
		# favoring the scale of each color to be similar  
		A2=np.array([[1.,0.,-1.,0.,0.,0.],[0.,0.,1.,0.,-1.,0.],[1.,0.,0.,0.,-1.,0.]])*diagonal_prior_weight
		b2=np.array([[0.0],[0.0],[0.0]])*diagonal_prior_weight
		# prior on scale,shift to be a,b
		A3=np.array([[1.,0.,0.,0.,0.,0.],[0.,0.,1.,0.,0.,0.],[0.,0.,0.,0.,1.,0.]])*a_prior_weight
		b3=np.array([[1.0],[1.0],[1.0]])*a_prior_weight
		# build matrices
		A=np.concatenate((A1,A2,A3),0)
		b=np.concatenate((b1,b2,b3),0)
		# solve equations
		# t1=time.time()
		p,residuals,rank,s=np.linalg.lstsq(A,b);
		# t2=time.time()
		# print("least-squares time: %f"%(t2-t1))
		fitting_cost=residuals
		RED_a=p[0]
		GREEN_a=p[2]
		BLUE_a=p[4]
		if (RED_a<0 or GREEN_a<0 or BLUE_a<0):
				fitting_cost=fitting_cost+INFEASIBILITY_PENALTY

		# Take the best solution if there were several permuations
		if (fitting_cost<min_fitting_cost):
			min_fitting_cost=fitting_cost
			#print("perm: %s, fitting cost: %s"% (perm,fitting_cost))
			MIN_RED_a=p[0]
			MIN_GREEN_a=p[2]
			MIN_BLUE_a=p[4]
			MIN_RED_b=p[1]
			MIN_GREEN_b=p[3]
			MIN_BLUE_b=p[5]
			min_perm=perm
	t2=time.time()
	# IPython.embed()


	#print MIN_RED_a, MIN_RED_b
	#print MIN_BLUE_a, MIN_BLUE_b
	#print MIN_GREEN_a, MIN_GREEN_b
	#print("time: %f"%(t2-t1))
	return ([MIN_RED_a], MIN_RED_b), ([MIN_BLUE_a], MIN_BLUE_b), ([MIN_GREEN_a], MIN_GREEN_b),fitting_cost

def getparameters(mapping, trained, true):
	redX = np.zeros((3, 1))
	redY = np.zeros((3, 1))
	greenX = np.zeros((3, 1))
	greenY = np.zeros((3, 1))
	blueX = np.zeros((3, 1))
	blueY = np.zeros((3, 1))
	# print type(redY), redX
	# print trained, true
	for i, color in enumerate(true):
		mapped = mapping[i]
		# print i, color
		for j, index in enumerate(color):
			# print index, j
			if j == 0:
				redY[i] = index
			if j == 1:
				greenY[i] = index
			if j == 2:
				blueY[i] = index
		for j2, index2 in enumerate(trained[mapped]):

			if j2 == 0:
				# print redX
				redX[i] = index2
				# print redX
			if j2 == 1:
				greenX[i] = index2
			if j2 == 2:
				blueX[i] = index2
	# IPython.embed()
	RED = linear_model.LinearRegression()
	BLUE = linear_model.LinearRegression()
	GREEN = linear_model.LinearRegression()
	RED.fit(redX, redY)
	BLUE.fit(blueX, blueY)
	GREEN.fit(greenX, greenY)
	fitting_cost=0 # should use score() from the regression
	# print RED_a_, RED_b
	# print BLUE_a_, BLUE_b
	# print GREEN_a_, GREEN_b
	return (RED.coef_, RED.intercept_), (BLUE.coef_, BLUE.intercept_), (GREEN.coef_, GREEN.intercept_),fitting_cost


if __name__ == '__main__':
	img_filename="test2.jpg"
	if (len(sys.argv)>1):
		img_filename=sys.argv[1]
		print(img_filename)
	cv_img = cv2.imread(img_filename)
	t1 = time.clock()
	testdata = getimgdatapts(cv_img)
	t2 = time.clock()
	print("Time taken:")
	print(t2-t1)
	
	trained = runKMeans(testdata)
	from anti_instagram import AntiInstagram
	mapping = identifyColors(trained[0], CENTERS)
	getparameters(mapping, trained[0], CENTERS)
