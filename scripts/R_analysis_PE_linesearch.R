error.bar <- function(x, y, upper, lower=upper, length=0.1,...){
if(length(x) != length(y) | length(y) !=length(lower) | length(lower) != length(upper))
stop("vectors must be same length")
arrows(x,y+upper, x, y-lower, angle=90, code=3, length=length, ...)
}

setwd("~/_Code/Multi-Robot-Search/build/")
temp = list.files(pattern="*.stats")
datadingoindex <- 0
vec <- c()
vec2 <- c()
n_files <- length(temp)
v_prev_l <- 6
costv <- c()
for (i in 1:n_files) {
  print(temp[i])
  temp_data <- read.csv(temp[i],header=FALSE)
  n_polies <- temp_data[1,1]
  n_fully_skipped <- temp_data[1,2]
  n_skipped <- temp_data[1,3]
  n_not_skipped <- temp_data[1,4]
  n_proper_avg <- temp_data[1,5]
  n_avg <- temp_data[1,6]
  hist <- temp_data[2,]
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
  if ( n_polies == 349) 
    n_polies = 350;
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
}

mat <- matrix(vec,nrow=2)
mat2 <- matrix(vec2,nrow=7)
costmat <- matrix(costv,nrow=2)

tmat2 <- t(mat2)
rawdata <- data.frame(tmat2);
f <- factor(tmat2[,1])
result2 <- tapply(rawdata$X2, f, mean)
result3 <- tapply(rawdata$X3, f, mean)
result4 <- tapply(rawdata$X4, f, mean)
result5 <- tapply(rawdata$X5, f, mean)
result6 <- tapply(rawdata$X6, f, mean)
result7 <- tapply(rawdata$X7, f, mean)

result2_normalized <- result2 / total_choicesets
result3_normalized <- result3 / total_choicesets
result4_normalized <- result4 / total_choicesets
result5_normalized <- result5 / total_choicesets
result6_normalized <- result6 / total_choicesets
result7_normalized <- result7 / total_choicesets


costmat2 <- t(costmat)
rawcost <- data.frame(costmat2);
cost_result <- tapply(rawcost$X2, f, mean)
plot(levels(f),cost_result,type="b",lty=1.0,pch=16,lwd=1,col="blue", ylim=c(0,400))

x1 <- as.numeric(levels(f))
y1.mean <- cost_result
y1.sd <- tapply(rawcost$X2, f, sd)
ylim <- c(0,400)
ylab <- "Cost"
xlab <- "Number of vertices"
samplesize <- 10
myPlot <- plot(x1,y1.mean,ylab=ylab,xlab=xlab,ylim=ylim,type="b",lty=0.5,pch=16,lwd=1,col="blue")
error.bar(x1,y1.mean, y1.sd / sqrt(samplesize),length=0.05,col="royalblue",lwd=0.5,lty=1)


total_choicesets = as.numeric(levels(f))*as.numeric(levels(f))
total_choicesets_f = as.factor(total_choicesets)

plot(levels(f),result2,type="b",lty=1.0,pch=16,lwd=1,col="blue", log='y', ylim=c(1,10000))
lines(levels(f),result3,type="b",lty=1.0,pch=15,lwd=1,col="green")
lines(levels(f),result4,type="b",lty=1.0,pch=15,lwd=1,col="yellow")
lines(levels(f),result5,type="b",lty=1.0,pch=15,lwd=1,col="black")
lines(levels(f),result6,type="b",lty=1.0,pch=15,lwd=1,col="red")
lines(levels(f),result7,type="b",lty=1.0,pch=15,lwd=1,col="orange")

plot(levels(f),result2_normalized,type="b",lty=1.0,pch=16,lwd=1,col="blue", log='y', ylim=c(0.000001,1))
lines(levels(f),result3_normalized,type="b",lty=1.0,pch=15,lwd=1,col="green")
lines(levels(f),result4_normalized,type="b",lty=1.0,pch=15,lwd=1,col="yellow")
lines(levels(f),result5_normalized,type="b",lty=1.0,pch=15,lwd=1,col="black")
lines(levels(f),result6_normalized,type="b",lty=1.0,pch=15,lwd=1,col="red")
lines(levels(f),result7_normalized,type="b",lty=1.0,pch=15,lwd=1,col="orange")