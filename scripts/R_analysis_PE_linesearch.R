error.bar <- function(x, y, upper, lower=upper, length=0.1,...){
if(length(x) != length(y) | length(y) !=length(lower) | length(lower) != length(upper))
stop("vectors must be same length")
arrows(x,y+upper, x, y-lower, angle=90, code=3, length=length, ...)
}

setwd("~/_Code/Multi-Robot-Search/build/data/")
temp = list.files(pattern="*.stats")
datadingoindex <- 0
vec <- c()
vec2 <- c()
n_files <- length(temp)
v_prev_l <- 6
costv <- c()
skipv <- c()
for (i in 1:n_files) {
  print(temp[i])
  temp_data <- read.csv(temp[i],header=FALSE)
  n_polies <- temp_data[1,1]
  n_fully_skipped <- temp_data[1,2]
  n_skipped <- temp_data[1,3]
  n_not_skipped <- temp_data[1,4]
  n_proper_avg <- temp_data[1,5]
  n_avg <- temp_data[1,6]
  hist <- temp_data[2,] #histogram of choice sets with 0,1,2,3 etc. cut sequences
  cost <- temp_data[3,1]
  v <- as.vector(t(hist))
  all_choice_sets <- n_fully_skipped;
  for ( j in 1:length(v)) {
    if ( !is.na(v[j]) ) {
      all_choice_sets <- all_choice_sets + v[j];
    } 
  }
  n_polies <- ceiling(sqrt(all_choice_sets))
  if ( n_polies == 199) 
    n_polies = 200;
  if ( n_polies == 299) 
    n_polies = 300;
  if ( n_polies == 349) 
    n_polies = 350;
  if ( n_polies == 399) 
    n_polies = 400;
  print(n_polies)
  vec <- c(vec, n_polies)
  vec <- c(vec, n_proper_avg)
  vec2 <- c(vec2, n_polies)
  vec2 <- c(vec2, v)
  if ( v_prev_l != length(v) ) {
    print("ERROR")
  }
  v_prev_l <- length(v)
  costv <- c(costv, n_polies)
  costv <- c(costv, cost)
  skipv <- c(skipv, n_polies)
  skipv <- c(skipv, n_fully_skipped)
}

mat <- matrix(vec,nrow=2)
mat2 <- matrix(vec2,nrow=7)
costmat <- matrix(costv,nrow=2)
skipmat <- matrix(skipv,nrow=2)

tmat2 <- t(mat2)
tmat2[is.na(tmat2)] <- 0
rawdata <- data.frame(tmat2);
f <- factor(tmat2[,1])
result2 <- tapply(rawdata$X2, f, mean,  na.rm=TRUE) # #of 0-size choice sequences
result3 <- tapply(rawdata$X3, f, mean,  na.rm=TRUE) # #of 1-size choice sequences
result4 <- tapply(rawdata$X4, f, mean,  na.rm=TRUE)
result5 <- tapply(rawdata$X5, f, mean,  na.rm=TRUE)
result6 <- tapply(rawdata$X6, f, mean,  na.rm=TRUE)
result7 <- tapply(rawdata$X7, f, mean,  na.rm=TRUE)

rawdata_n_choicesets = rawdata$X3 + rawdata$X4 *2 + rawdata$X5 *3 + rawdata$X6 *4 + rawdata$X7 *5

result2.sd <- tapply(rawdata$X2, f, sd,  na.rm=TRUE) # #of 0-size choice sequences
result3.sd <- tapply(rawdata$X3, f, sd,  na.rm=TRUE) # #of 1-size choice sequences
result4.sd <- tapply(rawdata$X4, f, sd,  na.rm=TRUE)
result5.sd <- tapply(rawdata$X5, f, sd,  na.rm=TRUE)
result6.sd <- tapply(rawdata$X6, f, sd,  na.rm=TRUE)
result7.sd <- tapply(rawdata$X7, f, sd,  na.rm=TRUE)

total_choicesets = as.numeric(levels(f))*as.numeric(levels(f))
total_choicesets_f = as.factor(total_choicesets)

result2_normalized <- result2 / total_choicesets
result3_normalized <- result3 / total_choicesets
result4_normalized <- result4 / total_choicesets
result5_normalized <- result5 / total_choicesets
result6_normalized <- result6 / total_choicesets
result7_normalized <- result7 / total_choicesets
result_total_cutsequences <- result3*1+result4*2+result5*3+result6*4+result7*5
result_total_cutsequences_normalized <- result_total_cutsequences / total_choicesets

costmat2 <- t(costmat)
skipmat2 <- t(skipmat)
rawcost <- data.frame(costmat2);
rawskip <- data.frame(skipmat2);
cost_result <- tapply(rawcost$X2, f, mean)
#plot(levels(f),cost_result,type="b",lty=1.0,pch=16,lwd=1,col="blue", ylim=c(0,400))

x1 <- as.numeric(levels(f))
y1.mean <- tapply(rawskip$X2, f, mean)
y1.sd <- tapply(rawskip$X2, f, sd)
ylim <- c(0,100000)
ylab <- "Skipped Choice Sets"
xlab <- "Number of vertices"
samplesize <- 10
#pdf("/Users/andreas/skip_across_size.pdf",height=6,width=6)
myPlot <- plot(x1,y1.mean,ylab=ylab,xlab=xlab,ylim=ylim,type="b",lty=0.5,pch=16,lwd=1,col="blue")
error.bar(x1,y1.mean, y1.sd / sqrt(samplesize),length=0.05,col="royalblue",lwd=0.5,lty=1)
#dev.off()

x1 <- as.numeric(levels(f))
y1.mean <- tapply(rawskip$X2, f, mean) / total_choicesets
y1.sd <- tapply(rawskip$X2, f, sd) / total_choicesets
ylim <- c(0,1)
ylab <- "Skipped Choice Sets"
xlab <- "Number of vertices"
samplesize <- 10
#pdf("/Users/andreas/skip_across_size.pdf",height=6,width=6)
myPlot <- plot(x1,y1.mean,ylab=ylab,xlab=xlab,ylim=ylim,type="b",lty=0.5,pch=16,lwd=1,col="blue")
error.bar(x1,y1.mean, y1.sd / sqrt(samplesize),length=0.05,col="royalblue",lwd=0.5,lty=1)
#dev.off()

pdf("/Users/andreas/skip_across_size.pdf",height=6,width=6)
par(mar=c(5.1,4.1,4.1,5.1))
x1 <- as.numeric(levels(f))
y1.mean <- tapply(rawskip$X2, f, mean)
y2 <- tapply(rawskip$X2, f, mean) / total_choicesets
ylab <- "Proportion of Skipped Choice Sets"
xlab <- "Number of vertices"
ylim <- c(0,1)
plot(x1, y2,ylab=ylab,xlab=xlab,type="b",lty=1,pch=16,lwd=1,col="blue")
par(new=TRUE)
ylab <- "Total Skipped Choice Sets"
ylim <- c(0,100000)
myPlot <- plot(x1,y1.mean,xaxt="n",yaxt="n",xlab="",ylab="",ylim=ylim,col="red",type="b")
axis(4)
par(new=FALSE)
mtext(ylab,side=4,line=3)
legend("topleft",col=c("blue","red"),lty=1,legend=c("Proportion","Total"))
dev.off()


#dev.off()

x1 <- as.numeric(levels(f))
y1.mean <- cost_result
y1.sd <- tapply(rawcost$X2, f, sd)
ylim <- c(0,400)
ylab <- "Cost (line clear number)"
xlab <- "Number of vertices"
samplesize <- 10
pdf("/Users/andreas/Cost_across_size.pdf",height=6,width=6)
myPlot <- plot(x1,y1.mean,ylab=ylab,xlab=xlab,ylim=ylim,type="b",lty=0.5,pch=16,lwd=1,col="blue")
error.bar(x1,y1.mean, y1.sd / sqrt(samplesize),length=0.05,col="royalblue",lwd=0.5,lty=1)
dev.off()

pdf("/Users/andreas/number_of_cutsequences_across_size.pdf",height=6,width=6)
x1 <- as.numeric(levels(f))
y1.mean <- y1.sd <- tapply(rawdata_n_choicesets, f, mean)
y1.sd <- tapply(rawdata_n_choicesets, f, sd)
ylim <- c(0,35000)
ylab <- "Number of sequences"
xlab <- "Number of vertices"
samplesize <- 10
myPlot <- plot(x1,y1.mean,ylab=ylab,xlab=xlab,ylim=ylim,type="b",lty=0.5,pch=16,lwd=1,col="blue")
error.bar(x1,y1.mean, y1.sd / sqrt(samplesize),length=0.05,col="royalblue",lwd=0.5,lty=1)
dev.off()

pdf("/Users/andreas/number_of_cutsequences_across_size.pdf",height=6,width=6)
x1 <- as.numeric(levels(f))
y1.mean <- y1.sd <- tapply(rawdata_n_choicesets, f, mean)
y1.sd <- tapply(rawdata_n_choicesets, f, sd)
ylim <- c(1,35000)
ylab <- "Number of sequences per choice set"
xlab <- "Number of vertices"
samplesize <- 10
par(mar=c(5.1,4.1,4.1,5.1))
#myPlot <- plot(x1,y1.mean,ylab=ylab,xlab=xlab,ylim=ylim,type="b",lty=0.5,pch=16,lwd=1,log="y",col="blue")
plot(x1, y2,ylab=ylab,xlab=xlab,type="b",lty=1,pch=16,lwd=1,col="blue")
par(new=TRUE)
y2 <- result_total_cutsequences_normalized;
ylab <- "Number of sequences in total"
myPlot <- plot(x1,y1.mean,xaxt="n",yaxt="n",xlab="",ylab="",ylim=ylim,col="red",type="b")
axis(4)
par(new=FALSE)
mtext(ylab,side=4,line=3)
legend("top",col=c("blue","red"),lty=1,legend=c("Number of sequences per choice set","Number of sequences in total"))

#myPlot <- plot(x1,sqrt(y1.mean),ylab=ylab,xlab=xlab,ylim=ylim,type="b",lty=0.5,pch=16,lwd=1,log="y",col="blue")
#error.bar(x1,y1.mean, y1.sd / sqrt(samplesize),length=0.05,col="royalblue",lwd=0.5,lty=1)
dev.off()
par(mar=c(5.1,4.1,4.1,2.1)

plot(levels(f),result3,type="b",lty=1.0,pch=16,lwd=1,col="blue", log='y', ylim=c(0.01,100000))
legend("topleft", c("Selection","Beacon"), cex=1.2, bty="n",fill=c("lightblue","red"));
lines(levels(f),result4,type="b",lty=1.0,pch=15,lwd=1,col="green")
lines(levels(f),result5,type="b",lty=1.0,pch=15,lwd=1,col="yellow")
lines(levels(f),result6,type="b",lty=1.0,pch=15,lwd=1,col="black")


plot(levels(f),result2,type="b",lty=1.0,pch=16,lwd=1,col="blue", ylim=c(0.01,40000))
lines(levels(f),result3,type="b",lty=1.0,pch=15,lwd=1,col="green")
lines(levels(f),result4,type="b",lty=1.0,pch=15,lwd=1,col="yellow")
lines(levels(f),result5,type="b",lty=1.0,pch=15,lwd=1,col="black")
lines(levels(f),result6,type="b",lty=1.0,pch=15,lwd=1,col="red")

ylab <- "Number of Choice Sets"
xlab <- "Number of vertices"
#names.arg=c("Stop**","Come***","Rendezvous","Deploy","Random***","Heading","Leave*"),col=c("lightblue","red"),cex.names=1.0,ylim=c(0,max(maxMode)),las = 1)
#legend("topleft", c("Selection","Beacon"), 
#cex=1.2, bty="n",fill=c("lightblue","red"));
#arrows(barx,maxMode, barx, minMode, angle=90, code=3, length=0.07)

plot(levels(f),result_total_cutsequences,type="b",lty=1.0,pch=16,lwd=1,col="blue",  ylim=c(0.1,100000))
plot(levels(f),sqrt(result_total_cutsequences),type="b",lty=1.0,pch=16,lwd=1,col="blue", ylim=c(0.1,400))
plot(levels(f),result_total_cutsequences,type="b",lty=1.0,pch=16,lwd=1,col="blue", log='y', ylim=c(0.1,100000))

plot(levels(f),result3,type="b",lty=1.0,pch=16,lwd=1,col="blue", log='y', ylim=c(0.1,100000)
#,legend("topleft", c("#1","#2","#3","#4","#5"), cex=1.2, bty="n",fill=c("lightblue","red"))
)
lines(levels(f),result4,type="b",lty=1.0,pch=15,lwd=1,col="green")
lines(levels(f),result5,type="b",lty=1.0,pch=15,lwd=1,col="yellow")
lines(levels(f),result6,type="b",lty=1.0,pch=15,lwd=1,col="black")
lines(levels(f),result7,type="b",lty=1.0,pch=15,lwd=1,col="red")

plot(levels(f),result2_normalized,type="b",lty=1.0,pch=16,lwd=1,col="blue", log='y', ylim=c(0.000001,1))
lines(levels(f),result3_normalized,type="b",lty=1.0,pch=15,lwd=1,col="green")
lines(levels(f),result4_normalized,type="b",lty=1.0,pch=15,lwd=1,col="yellow")
lines(levels(f),result5_normalized,type="b",lty=1.0,pch=15,lwd=1,col="black")
lines(levels(f),result6_normalized,type="b",lty=1.0,pch=15,lwd=1,col="red")
lines(levels(f),result7_normalized,type="b",lty=1.0,pch=15,lwd=1,col="orange")

table(rawcost$X2)