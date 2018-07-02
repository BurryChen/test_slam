import numpy  
my_matrix = numpy.loadtxt(open("/media/whu/Research/04Research_PhD/01LRF_Calibation/data/dataset4_angle90_45_20180619/tif_table/residual_check.csv","rb"),delimiter=",",skiprows=0) 

#print my_matrix
mean=numpy.mean(my_matrix,axis=0)
print mean
print numpy.rms(my_matrix[:,1])
#print("RMSE = ", sqrt(sum(squaredError) / len(squaredError)))
