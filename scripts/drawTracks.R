#!/usr/local/bin/Rscript

args = commandArgs(trailingOnly=TRUE)

if (length(args)==0) {
  stop("Usage: drawGraphs.R csv-file", call.=FALSE)
} 

#Read data
csv <- read.csv(file=args[1], head=TRUE, sep=",")

path = paste("test_", format(Sys.time(), "%d_%b_%Y-%H-%M-%S"), sep="")
dir.create(file.path(".", path))
samples = length(csv[,1])

lastStateAt = 6
polynomialGrade = 4
waypoints = 6
beginWaypoints = lastStateAt+polynomialGrade+1
beginPredictions = beginWaypoints+2*waypoints
predictions = 14

pos_x = as.numeric(as.character(csv[,1]))
pos_y = as.numeric(as.character(csv[,2]))
psi = as.numeric(as.character(csv[,3]))
v = as.numeric(as.character(csv[,4]))
cte = as.numeric(as.character(csv[,5]))
epsi = as.numeric(as.character(csv[,6]))
#delta = as.numeric(as.character(csv[,7]))
#a = as.numeric(as.character(csv[,8]))

coeff = matrix(unlist(csv[,(lastStateAt+1):(lastStateAt+polynomialGrade)]), ncol = polynomialGrade)
w_x = matrix(unlist(csv[,(beginWaypoints):(beginWaypoints+waypoints-1)]), ncol = waypoints)
w_y = matrix(unlist(csv[,(beginWaypoints+waypoints):(beginWaypoints+(2*waypoints)-1)]), ncol = waypoints) 
p_x = matrix(unlist(csv[,beginPredictions:(beginPredictions+predictions-1)]), ncol = predictions)
p_y = matrix(unlist(csv[,(beginPredictions+predictions):(beginPredictions+(2*predictions)-1)]), ncol = predictions)

img_h = 960
img_w = 960

file_name = paste("./", path, sep="")
file_name = paste(file_name, "/track", sep="")

for (i in 1:samples)
{
	curr_file = paste(file_name, i, sep="")
	curr_file = paste(curr_file, ".png", sep="")
	png(curr_file, width = img_w, height = img_h)

	plot(w_x[i,], w_y[i,], col="blue", lwd=1, xlim=c(-10,100) , ylim=c(-10,100))
	curve(coeff[i,1]+coeff[i,2]*x+coeff[i,3]*x^2+coeff[i,4]*x^3, from=0, to=80, lty=2, xlim=c(-10,100), ylim=c(-10,100), ylab='y')
	points(pos_x[i],pos_y[i], type="p")

	points(p_x[i,], p_y[i,], col="red", lwd=1)
	lines(p_x[i,], p_y[i,], col="red", lwd=1)

	points(w_x[i,], w_y[i,], col="blue", pch=16 )
	lines(w_x[i,], w_y[i,], col="blue", lwd=1)
	
	arrows(pos_x[i],pos_y[i],pos_x[i]+5,pos_y[i], length = 0.2, angle = 20, col="red", lwd=2)
	
	dev.off()
}
