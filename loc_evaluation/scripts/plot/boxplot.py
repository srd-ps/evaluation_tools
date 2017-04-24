import numpy as np
from pylab import *

fullipatrans = open("./fullIPA/localgrid/trans23.txt")
fulliparot = open("./fullIPA/localgrid/rot23.txt")
amcltrans = open("./amcl/trans23.txt")
amclrot = open("./amcl/rot23.txt")
linestrans = open("./lines/trans23.txt")
linesrot = open("./lines/rot23.txt")
pointstrans = open("./points/trans23.txt")
pointsrot = open("./points/rot23.txt")
linesposestrans = open("./linesposes/localgrid/trans23.txt")
linesposesrot = open("./linesposes/localgrid/rot23.txt")

bp=[]
d=[]

numCol = 5
numRow = 2


d.append(np.absolute(map(double, amcltrans.readlines()))) 		#0
d.append(np.absolute(map(double, fullipatrans.readlines()))) 	#1
d.append(np.absolute(map(double, linestrans.readlines())))		#2
d.append(np.absolute(map(double, pointstrans.readlines())))		#3
d.append(np.absolute(map(double, linesposestrans.readlines()))) #4
d.append(np.absolute(map(double, amclrot.readlines())))			#5
d.append(np.absolute(map(double, fulliparot.readlines())))		#6
d.append(np.absolute(map(double, linesrot.readlines())))		#7
d.append(np.absolute(map(double, pointsrot.readlines())))		#8
d.append(np.absolute(map(double, linesposesrot.readlines())))	#9

fig, ax = subplots(numRow, numCol, figsize=(15,8))

subplots_adjust(left=0.10, right=0.90, top=0.90, bottom=0.17)

#ax[0][2].set_title('Comparison of different configurations')

ax[0][0].set_xticklabels([""])
ax[0][1].set_xticklabels([""])
ax[0][2].set_xticklabels([""])
ax[0][3].set_xticklabels([""])
ax[0][4].set_xticklabels([""])
ax[1][0].set_xticklabels(["AMCL"])
ax[1][1].set_xticklabels(["all features combined"])
ax[1][2].set_xticklabels(["line features"])
ax[1][3].set_xticklabels(["point features"])
ax[1][4].set_xticklabels(["line and pose features"])

for i in range(numRow):
	for j in range(numCol):
		bp.append(ax[i][j].boxplot(d[numCol*i+j]))
		ax[i][j].yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=1)
		setp(bp[numCol*i+j]['boxes'], color='black')
		setp(bp[numCol*i+j]['whiskers'], color='black')
		setp(bp[numCol*i+j]['fliers'], color='red', marker='+')
		if (j!=0):
			ax[i][j].set_yticklabels("")
		if(i==0):
			ax[i][j].set_ylim(0, 0.21)
		else:
			ax[i][j].set_ylim(0, 8)

for i in range(numRow):

	for j in range(numCol):
		boxX = []
		boxY = []

		for k in range(5):
			boxX.append(bp[numCol*i+j]["boxes"][0].get_xdata()[k])
			boxY.append(bp[numCol*i+j]["boxes"][0].get_ydata()[k])

		boxCoords = zip(boxX,boxY)
		if (j==0):
			boxPolygon = Polygon(boxCoords, facecolor="#99CCFF")
		elif (j==1 or j==4):
			boxPolygon = Polygon(boxCoords, facecolor="#FF9966")
		else:
			boxPolygon = Polygon(boxCoords, facecolor="#FFFF99")
			
		ax[i][j].add_patch(boxPolygon)

#version 1
#plt.figtext(0.105, 0.045,  "Adaptive Monte Carlo Localization" , backgroundcolor="#99CCFF", color='black', weight='roman', size='x-small')
#plt.figtext(0.30, 0.045, "Feature Navigation", backgroundcolor="#FFFF99", color='black', weight='roman', size='x-small')
#plt.figtext(0.468, 0.045, "Feature Navigation", backgroundcolor="#FFFF99", color='black', weight='roman', size='x-small')
#plt.figtext(0.628, 0.045, "Feature Navigation", backgroundcolor="#FFFF99", color='black', weight='roman', size='x-small')
#plt.figtext(0.798, 0.045, "Feature Navigation", backgroundcolor="#FFFF99", color='black', weight='roman', size='x-small')

#version 2
plt.figtext(0.10, 0.100,  "Adaptive Monte Carlo Localization",     backgroundcolor="#99CCFF", color='black', weight='roman', size='12')
plt.figtext(0.10, 0.060,  "Feature Localization with local grid",    backgroundcolor="#FF9966", color='black', weight='roman', size='12')
plt.figtext(0.10, 0.020,  "Feature Localization without local grid", backgroundcolor="#FFFF99", color='black', weight='roman', size='12')

ax[0][0].set_ylabel("translational error (m)")
ax[1][0].set_ylabel("rotational error (degree)")

show()