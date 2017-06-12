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

pos_x = as.numeric(as.character(csv[,1]))
pos_y = as.numeric(as.character(csv[,2]))
psi = as.numeric(as.character(csv[,3]))
v = as.numeric(as.character(csv[,4]))
cte = as.numeric(as.character(csv[,5]))
epsi = as.numeric(as.character(csv[,6]))
#delta = as.numeric(as.character(csv[,7]))
#a = as.numeric(as.character(csv[,8]))

w_x = matrix(unlist(csv[,7:12]), ncol = 6)
w_y = matrix(unlist(csv[,13:18]), ncol = 6) 

p_x = matrix(unlist(csv[,19:33]), ncol = 15)
p_y = matrix(unlist(csv[,34:48]), ncol = 15)

img_h = 960
img_w = 960

file_name = paste("./", path, sep="")
file_name = paste(file_name, "/track", sep="")

for (i in 1:samples)
{
	curr_file = paste(file_name, i, sep="")
	curr_file = paste(curr_file, ".png", sep="")
	png(curr_file, width = img_w, height = img_h)

	plot(w_x[i,], w_y[i,], col="blue", lwd=1, xlim=c(-5,100) , ylim=c(-5,100))
	points(pos_x[i],pos_y[i], type="p")

	points(p_x[i,], p_y[i,], col="red", lwd=1)
	lines(p_x[i,], p_y[i,], col="red", lwd=1)

	points(w_x[i,], w_y[i,], col="blue", pch=16 )
	lines(w_x[i,], w_y[i,], col="blue", lwd=1)
	
	arrows(pos_x[i],pos_y[i],pos_x[i]+5,pos_y[i], length = 0.2, angle = 20, col="red", lwd=2)
	
	dev.off()
}
