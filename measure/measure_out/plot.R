library(ggplot2)
library(ggpubr)
library(latex2exp)

path <- dirname(rstudioapi::getSourceEditorContext()$path)
data.c <- read.table(paste(path, '/clustering.dat', sep=''), header=F, fill=T)
names(data.c) <- c('rho', 'r', 'h')
p1 <- ggplot(data=data.c, aes(x=r, y=h, group=rho)) + 
  theme_light() + 
  theme(legend.position = c(.95,.7), legend.background = element_rect(fill=alpha('white', 0.))) +
  guides(colour = guide_colourbar(title.hjust = 0.8, label.position = 'left')) +
  geom_line(aes(color=rho)) + 
  labs(colour=TeX("$\\rho$"), x="r", y=TeX("$H_{\\rho}(r)$"))

data.cmax <- (sapply(split(data.c, data.c$rho), function(s) s[which.max(s$h), c(1,3)] ))
data.cmax <- data.frame(matrix(unlist(data.cmax), ncol=2, nrow=length(data.cmax)/2, byrow=T))
names(data.cmax) <- c('rho', 'max')
p2 <- ggplot(data=data.cmax, aes(x=rho, y=max)) + 
  theme_light() + 
  geom_line() + 
  scale_x_continuous(breaks=seq(0,10,1)/10) +
  labs(x=TeX("$\\rho$"), y=TeX("$H_{max}(\\rho)$")) 

p.c <- ggarrange(p1, p2, labels="auto", ncol=1, nrow=2)
ggsave(paste(path, '/plot_h.pdf', sep=''), plot=p.c)

data.v <- read.table(paste(path, '/velocity.dat', sep=''), header=F, fill=T)
p <- ggplot(data=data.v, aes(x=V1, y=V2)) + 
  theme_light() +
  geom_line() +
  geom_point()+
  geom_errorbar(aes(ymin=V2-V3, ymax=V2+V3), width=.01, position=position_dodge(0.005)) +
  scale_x_continuous(breaks=seq(0,10,1)/10) +
  #scale_x_continuous(trans='log10') + 
  labs(title="Average velocity", x=TeX("$\\rho$"), y ="v")
ggsave(paste(path, '/plot_v.pdf', sep=''), plot=p)