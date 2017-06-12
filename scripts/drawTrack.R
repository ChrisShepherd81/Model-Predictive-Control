#!/usr/local/bin/Rscript

#Read data
csv <- read.csv(file="../lake_track_waypoints.csv", head=TRUE, sep=",")

pos_x = as.numeric(as.character(csv[,1]))
pos_y = as.numeric(as.character(csv[,2]))

img_h = 960
img_w = 960

png("track.png", width = img_w, height = img_h)

cap_x = c(-32.16173,-43.49173,-61.09,-78.29172,-93.05002,-107.7717)
cap_y = c(113.361,105.941,92.88499,78.73102,65.34102,50.57938)
car_x = -40.62
car_y = 108.73
psi = 3.733651
plot(pos_x,pos_y, type="p", pch=4)
points(cap_x, cap_y, pch=16 )

lines(cap_x, cap_y, col="blue", lwd=2)

points(car_x, car_y, col="red", lwd=2)
arrows(car_x, car_y, car_x+(10*cos(psi)), car_y+(10*sin(psi)), length = 0.2, angle = 20, col="red", lwd=2)

dev.off()

png("track_zoom.png", width = img_w, height = img_h)

plot(cap_x,cap_y, type="p", pch=16)
lines(cap_x, cap_y, col="blue", lwd=2)
points(car_x, car_y, col="red", lwd=2)
arrows(car_x, car_y, car_x+(10*cos(psi)), car_y+(10*sin(psi)), length = 0.2, angle = 20, col="red", lwd=2)
dev.off()

rot_x = c(0,13.5424,35.4315,57.6043,77.3232,97.7772)
rot_y = c(0,-0.165866,0.846859,2.99197,5.86674,9.90012)

png("track_rot.png", width = img_w, height = img_h)
plot(rot_x,rot_y, type="p", pch=16, ylim=c(-10,100),xlim=c(-10,100) )
lines(rot_x, rot_y, col="blue", lwd=2)
dev.off()
