setwd( "~/projects/mapper.git/software/.build.app" )
workOrigData <- read.table( './data/points3d.txt' )
names( workOrigData ) <- c( 'x', 'y', 'z', 'cx', 'cy', 'cz' )

library( plot3D )
library( plot3Drgl )

d<-workOrigData
plot3d( d$x, d$y, d$z )
