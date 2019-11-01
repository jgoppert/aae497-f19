import matplotlib.pyplot as plt 

time_quad_search = [70592,33250,35424,18072,15555,16509,32368,12693,11016,20349,52016,33128,11111,41022,35417,12469,38529,31943,12439,10874,38309,35429,11781,10704,13960,14161,29970,34661,31576,13384,11129,15665,47686,34592,11914,14982,36714,12914,18757,12917]

res = range(1,41)

plt.scatter(time_quad_search,res,marker='o')

plt.xlabel('Computation time in ns') 
plt.ylabel('Resolution') 
  
plt.title('Plot of Computation time in ns vs Resolution')  
plt.show() 