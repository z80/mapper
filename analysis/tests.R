setwd( "~/projects/mapper.git/software/.build.app" )
workOrigData <- read.csv( './data/points3d.txt', header=F,sep=";" )
names( workOrigData ) <- c( 'x', 'y', 'z', 'cx', 'cy', 'cz' )

library( plot3D )
library( plot3Drgl )

d<-workOrigData
d$z <- -as.numeric(d$z)
plot3d( d$x, d$y, d$z )
