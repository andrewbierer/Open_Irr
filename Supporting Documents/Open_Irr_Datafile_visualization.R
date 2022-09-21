# 
# MIT License
# 
# Copyright (c) 2022 Andrew Bierer
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#   
#   The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


# This script is intended to visualize  Open_Irr data files.
# this may be made into an Rmd file in the future..
library(tidyverse)
library(readxl)
library(rcompanion)
library(cowplot)
library(gridGraphics)
library(gridExtra)
library(gtable)
library(ggsci)
library(reshape)
library(reshape2)
library(lubridate)

vizualize_sensors <- function(file_path, start_line, number_wm, number_temp, WM_ON=NULL, gateway_file=NULL, view_window_start=NULL, view_window_end=NULL){
 
  if(is.null(gateway_file)){ #If gateway file is not utilized then do this.
    my_files <- list.files(path = paste(file_path), pattern = "*.TXT", full.names = TRUE)
    #"N:/Scientist/ABierer/Projects/Open Irr/preliminary test, January 2022/Raw_Data/2022_06_23/"
    my_data <- as.list(my_files)
    
    my_data <- do.call(bind_rows, lapply(my_data, read.csv2, skip=7, header=FALSE)) #All data in 1 column
    
    #start_line <- 9
    
    ncolumns <- sapply(my_files, function(f) {
      data.start.line <- readLines(f, n=start_line)
      dat_length <- length(strsplit(data.start.line, ",")[[start_line]])
      
      end_line <- fpeek::peek_count_lines(f, with_eof = FALSE)
      data.end.line <- readLines(f, end_line)
      
      end_line_length <- length(strsplit(data.end.line, ",")[[end_line]])
      if(end_line_length > dat_length){return(end_line_length)}else{return(dat_length)}
    })
    
    length_header <- max(ncolumns) # this solution polls each of the files in the list and checks start line and line 1
    
    
    str_split_fixed(my_data[,1], pattern=",", n=length_header)
    
    #number_wm <- 16
    #number_temp <- 6
    
    channel_info <- rep(c("mux_channel", "resistance_ohms", "matric_potential_kpa"), times=number_wm)
    temp_info <- rep(c("temperature_sensor", "temperature_c"), times=number_temp)
    
    channel_info <- paste(rep("mux_channel", times=number_wm), ".", 0:(number_wm-1))
    channel_info <- gsub(" ", "", channel_info, fixed = TRUE)
    
    resistance_info <- paste(rep("resistance_ohms", times=number_wm), ".", 0:(number_wm-1))
    resistance_info <- gsub(" ", "", resistance_info, fixed = TRUE)
    
    matric_info <- paste(rep("matric_potential_kpa", times=number_wm), ".", 0:(number_wm-1))
    matric_info <- gsub(" ", "", matric_info, fixed = TRUE)
    
    temp_sensor_info <- paste(rep("temperature_sensor", times=number_temp),".",0:(number_temp-1))
    temp_sensor_info <- gsub(" ","", temp_sensor_info, fixed = TRUE)
    
    temp_c_info <- paste(rep("temperature_c", times=number_temp),".",0:(number_temp-1))
    temp_c_info <- gsub(" ","", temp_c_info, fixed = TRUE)
    
    wm_info <- c(rbind(channel_info,resistance_info,matric_info))
    temp_info <- c(rbind(temp_sensor_info,temp_c_info))
    
    group_num <- c(paste(rep("Group_wm", times=4),".",0:3)) # times - 1
    group_num <- gsub(" ","", group_num, fixed = TRUE)
    
    group_kpa <- c(paste(rep("Group_kpa", times=4),".",0:3))
    group_kpa <- gsub(" ","", group_kpa, fixed = TRUE)
    
    group_last_irr <- c(paste(rep("Group_last_irr", times=4),".",0:3))
    group_last_irr <- gsub(" ","", group_last_irr, fixed = TRUE)
    
    wm_info <- c(rbind(channel_info,resistance_info,matric_info))
    temp_info <- c(rbind(temp_sensor_info,temp_c_info))
    group_info <- c(rbind(group_num,group_kpa,group_last_irr))
    
    
    if(is.null(WM_ON)){
      header_names <- c("node_id", "node_voltage", "datetime", wm_info, temp_info)
    } else {
      header_names <- c("node_id", "node_voltage", "datetime", wm_info, group_info, temp_info)
    }
    
    my_data <- as.data.frame(str_split_fixed(my_data[,1], pattern = ",", n=length_header)) #only uses a fixed column length
    colnames(my_data) <- header_names
    
    my_data$record <- seq(1:dim(my_data)[1]) #specify a unique value for each row, here the sequence of row numbers
   
  #  test.data <- my_data
  #  my_data <- test.data
    
  ### Save Wide-format data to file to access later for analysis 6/24/2022 ###
  #  write_csv(my_data, file = "N:/Scientist/ABierer/Projects/Open Irr/preliminary test, January 2022/Parsed_Data/node_wide.csv")
    
     
    my_data <- my_data %>%
      pivot_longer(names_to = "variable", values_to = "value", -c(record, node_id, node_voltage, datetime))%>%
      separate(variable, c("variable", "sensor"), sep = "[.]", fill = "right", extra = "merge", convert = TRUE)%>%
      pivot_wider(names_from = variable, values_from = value, values_fill = NA)

    ###
    my_data$datetime <- as.POSIXct(my_data$datetime, tz="EST", format="%m/%d/%Y %H:%M:%S")
    my_data$Group_last_irr <- as.POSIXct(my_data$Group_last_irr, tz="EST", format="%m/%d/%Y %H:%M:%S")
    my_data$Group_kpa <- as.numeric(my_data$Group_kpa)
    my_data$Group_wm <- as.factor(my_data$Group_wm)
    my_data$matric_potential_kpa <- as.numeric(my_data$matric_potential_kpa)
    my_data$mux_channel <- as.factor(my_data$mux_channel)
    my_data$node_voltage <- as.numeric(my_data$node_voltage)
    my_data$matric_potential_kpa[my_data$matric_potential_kpa > 0] <- 0 #Replace any positive matric potential values with 0
    my_data$node_id <- as.numeric(as.character(my_data$node_id))
    my_data <- my_data%>%
      filter(., node_id <= 105 & node_id >= 101)
    my_data$node_id <- as.factor(my_data$node_id)
    ###
    
   
    
    #Zoom to datetime of interest as specified in the function
    #default to viewing all data
    if(is.null(view_window_start) | is.null(view_window_end)){
      #null_start <- (max(my_data$datetime) - lubridate::hours(48))
      #null_end <- max(my_data$datetime)
      #null_interval <- lubridate::interval(start = null_start, end = null_end, tz="EST) # define time window
      my_data <- my_data #[my_data$datetime %within% null_interval,] # return values within the time window
    } else {
      selected_interval <- lubridate::interval(start = view_window_start, end = view_window_end, tz="EST")
      my_data <- my_data[my_data$datetime %within% selected_interval,]
    }
    #G1 to 4 & NA present
 # rat <-  as.data.frame(my_data %>%
 #      group_by(node_id, Group_wm)%>% #group by node & treatment (irr group here)
 #     summarize(N_irr_events = sum(diff(Group_last_irr)!=0),
 #                last_irr_event = max(Group_last_irr)))
  #this looks correct here, but not sorted by treatment (although Group_wm is essentially the same...)
    
    #Add case when for sorting.
    
    my_data <- my_data %>%
      rowwise()%>%
      mutate(treatment = case_when(
        ((mux_channel == 4 | mux_channel ==5 | mux_channel ==11) &  (node_id == 101))  ~  "Control",
        ((mux_channel == 6 | mux_channel == 8 | mux_channel == 9) & (node_id == 102)) ~ "Control",
        ((mux_channel == 3 | mux_channel == 7 | mux_channel == 9) & (node_id == 103)) ~ "Control",
        ((mux_channel == 7 | mux_channel == 8 | mux_channel == 11) & (node_id == 104)) ~ "Control",
        ((mux_channel == 4 | mux_channel == 7 | mux_channel == 10) & (node_id == 105)) ~ "Control",
        
        ((mux_channel == 6 | mux_channel == 8 | mux_channel == 12) & (node_id == 101)) ~  "-50 kPa",
        ((mux_channel == 4 | mux_channel == 7 | mux_channel == 11) & (node_id == 102)) ~ "-50 kPa",
        ((mux_channel == 1 | mux_channel == 2 | mux_channel == 6) & (node_id == 103)) ~ "-50 kPa",
        ((mux_channel == 4 | mux_channel == 5 | mux_channel == 10) & (node_id == 104)) ~ "-50 kPa",
        ((mux_channel == 2 | mux_channel == 8 | mux_channel == 11) & (node_id == 105))  ~ "-50 kPa",
        
        ((mux_channel == 1 | mux_channel == 7 | mux_channel == 9) & (node_id == 101)) ~ "-100 kPa",
        ((mux_channel == 1 | mux_channel == 10 | mux_channel == 12) & (node_id == 102)) ~ "-100 kPa",
        ((mux_channel == 5 | mux_channel == 8 | mux_channel == 11) & (node_id == 103)) ~ "-100 kPa",
        ((mux_channel == 3 | mux_channel == 6 | mux_channel == 12) & (node_id == 104))  ~ "-100 kPa",
        ((mux_channel == 1 | mux_channel == 6 | mux_channel == 9) & (node_id == 105)) ~ "-100 kPa",
        
        ((mux_channel == 2 | mux_channel == 3 | mux_channel == 10) & (node_id == 101)) ~ "-200 kPa",
        ((mux_channel == 2 | mux_channel == 3 | mux_channel == 5) & (node_id == 102)) ~ "-200 kPa",
        ((mux_channel == 4 | mux_channel == 10 | mux_channel == 12) & (node_id == 103)) ~ "-200 kPa",
        ((mux_channel == 1 | mux_channel == 2 | mux_channel == 9) & (node_id == 104)) ~ "-200 kPa",
        ((mux_channel == 3 | mux_channel == 5 | mux_channel == 12) & (node_id == 105)) ~ "-200 kPa",
        
        TRUE ~ "Error"
      )) # did not mess up irr summary, all groups present 
    unfiltered_data <-  my_data #for grabbing from unaligned group_wm data.
    
    ### Save Long-format data to file to access later for analysis 6/24/2022 ###
    write_csv(my_data, file = "N:/Scientist/ABierer/Projects/Open Irr/preliminary test, January 2022/Parsed_Data/node_long.csv")
    # my_data <- read_csv(file = "N:/Scientist/ABierer/Projects/Open Irr/preliminary test, January 2022/Parsed_Data/node_long.csv")
    
    ######below
    unfiltered_data <- unfiltered_data %>%
      filter(., Group_wm == "G1" || Group_wm == "G2" || Group_wm =="G3" || Group_wm =="G4") # removing partial transmissions from the gateway until its fixed in program...
    unfiltered_data <- unfiltered_data%>%
      filter(., Group_kpa >= -255 && Group_kpa <= 0) #again removing partial transmissions.
    
    #unfiltered_data <- unfiltered_data%>%
    #  filter(treatment != "Error")
    
    my_data <- my_data%>%
      filter(treatment != "Error")
    
    my_data <- my_data%>%
      filter(matric_potential_kpa <= 0)
    
    my_data$treatment <- as.factor(my_data$treatment)
    
  #  my_data%>%
  #    group_by(node_id, Group_wm)%>%
  #    summarize(unique_channels = unique(as.numeric(.$mux_channel)))
    
   # unfiltered_data <-  my_data
   # 
   #  my_data <- my_data%>%
   #    filter(treatment != "Error") ###This is the call that removed the G1 from the dataframe. why?
   #  #-> It is because while all desired data is present in the dataframe before this filter, It does not
   #  #necessarily correspond across unrelated columns. The Group_wm correctly has the group number & mean
   #  # in Group_kpa & time in Group_last_irr , BUTTTT This does not align (&or repeated) over the mux_channels
   #  # belonging to each group. Best solution is to specify an un-altered data frame to reference later in the 
   #  # function for visualizing based on Group_wm data. Otherwise, all three (Group_wm, Group_kpa, Group_last_irr)
   #  # would need attributed to each timed reference for each mux_channel.
   #  
   #  ## Will be called unfiltered_data
   #  my_data <- my_data%>%
   #    filter(matric_potential_kpa <= 0)
   #  
   #  my_data$treatment <- as.factor(my_data$treatment)
    
    # Make Graphs of interest  
    each_mux_channel  <- ggplot(my_data)+
      geom_line(mapping = aes(x=datetime, y=matric_potential_kpa, group=mux_channel, color=mux_channel))+
      scale_x_datetime()+
      scale_y_continuous(limits = c(-250,0))+
      scale_color_igv()+
      facet_wrap("node_id")
    
    by_treatment_facet <-  ggplot(my_data)+
      geom_point(mapping = aes(x=datetime, y=matric_potential_kpa, group=treatment, color=treatment))+
      scale_x_datetime()+
      scale_y_continuous(limits = c(-250,0))+
      scale_color_npg()+
      facet_wrap("node_id")
    
    by_treatment_no_facet <-  ggplot(my_data)+
      geom_point(mapping = aes(x=datetime, y=matric_potential_kpa, group=treatment, color=treatment))+
      scale_x_datetime()+
      scale_y_continuous(limits = c(-250,0))+
      scale_color_npg()
    
    fun0 <- function(x) 0
    
    if(is.null(view_window_start) | is.null(view_window_end)){
      graph_label_position <- as.POSIXct(max(my_data$datetime), tz="EST") 
    } else {
      graph_label_position <-as.POSIXct(view_window_end, tz="EST") 
    }

    
    treatment_means_no_facet_data_table <- function(){
      func_data <- my_data %>%
        group_by(treatment)%>%
        summarize(treatment_mean = mean(matric_potential_kpa),
                  treatment_stdev = sd(matric_potential_kpa))
      
      func_table <- tableGrob(as.data.frame(func_data), rows = NULL, theme = ttheme_default(base_size=10))
      return(func_table)
    }
    
    treatment_means_no_facet <- ggplot(my_data)+
      geom_point(mapping = aes(x=datetime, y=matric_potential_kpa, group=treatment, color=treatment), stat= "summary", fun="mean")+
      stat_summary(fun = 'mean', geom = 'text', aes(x = as.POSIXct(graph_label_position), y= matric_potential_kpa, label=paste("mean:",signif(..y..,4)), group=treatment, color=treatment), position = position_nudge(x=12*60*60), show.legend=FALSE)+ # make the call to x position the end date window
      stat_summary(fun = 'mean', fun.min = 'sd', geom = 'text', aes(x = as.POSIXct(graph_label_position), y=matric_potential_kpa, label=paste("sd:", signif(..ymin..,4)), group=treatment, color=treatment), position = position_nudge(x=24*60*60), show.legend=FALSE)+ #x=as.integer(treatment)
      annotation_custom(treatment_means_no_facet_data_table(), xmin = graph_label_position - (24*60*60), xmax = graph_label_position - (60*60), ymin = -100, ymax = -75)+
      scale_x_datetime()+
      scale_y_continuous(limits = c(-250,0))+
      scale_color_npg()
    
    #Do treatment means except by group means where the mean is buffered against outliers by the system
    
    group_means_no_facet_data_table <- function(){
      func_data <- unfiltered_data%>%
        group_by(Group_wm)%>%
        na.omit()%>%
        summarize(Group_wm_mean = mean(Group_kpa),
                  Group_wm_stdev = sd(Group_kpa))
      func_table <- tableGrob(as.data.frame(func_data), rows = NULL, theme = ttheme_default(base_size = 10))
      return(func_table)
    }
    
    group_means_no_facet <- ggplot(unfiltered_data)+
      geom_point(mapping = aes(x=datetime, y=Group_kpa, group=Group_wm, color=Group_wm), stat = "summary", fun="mean")+
      stat_summary(fun = 'mean', geom = 'text', aes(x = as.POSIXct(graph_label_position), y= Group_kpa, label=paste("mean:",signif(..y..,4)), group=Group_wm, color=Group_wm), position = position_nudge(x=12*60*60), show.legend=FALSE)+ # make the call to x position the end date window
      stat_summary(fun = 'mean', fun.min = 'sd', geom = 'text', aes(x = as.POSIXct(graph_label_position), y=Group_kpa, label=paste("sd:", signif(..ymin..,4)), group=Group_wm, color=Group_wm), position = position_nudge(x=24*60*60), show.legend=FALSE)+ #x=as.integer(treatment)
      annotation_custom(group_means_no_facet_data_table(), xmin = graph_label_position - (24*60*60), xmax = graph_label_position - (60*60), ymin = -100, ymax = -75)+
      scale_x_datetime()+
      scale_y_continuous(limits = c(-250,0))+
      scale_color_npg()
    
    #Vizualize Battery Levels
    battery_levels <- function(){
      
      func_data <- my_data%>%
        group_by(node_id)%>%
        filter(datetime == max(datetime))%>%
        summarise(node_voltage = min(node_voltage)) # minimum voltage at most recent datetime for each node
      
      
      battery_func_plot <- ggplot(func_data)+
        geom_col(mapping = aes(x=node_id, y=node_voltage, group=node_id, fill=node_id))+
        geom_abline(intercept = 6.5, slope = 0, linetype=2, color="red")+
        annotate("text", x=1.75, y=6.0, label="Change battery pack below this line.")+
        scale_color_npg()
      
      
      return(battery_func_plot)
    }
    
    
    #max(my_data$datetime) - min(my_data$datetime)
    
    #Count & display number of irrigation events and the last irrigation event to occur for each group/node
    irrigation_info <- function(){
      func_data <- unfiltered_data %>%
        group_by(node_id, Group_wm)%>%
        filter(Group_wm == "G1" || Group_wm =="G2"||Group_wm=="G3"||Group_wm=="G4")%>%
        slice_max(Group_last_irr, n = 1, with_ties = FALSE) # finds the max, here latest time, of Group_last_irr in the dataframe
      
      interval_test <- outer(func_data$Group_last_irr, unfiltered_data$datetime, "-") # get time difference between (x,y)
      interval_test[interval_test < 0] <- NA # remove where datetime is after Group_last_irr, leaving values before Group_last_irr 
      ind <- apply(interval_test, 1, function(i) which.min(i)) # find index of minimum, where the time between / interval is the smallest
      interval_out <- cbind(func_data, unfiltered_data[ind,]) #output data
      #interval_out
      match(as.POSIXlt(interval_out$datetime...3), as.POSIXlt(unfiltered_data$datetime)) #not sure if this, 2 less than gateway file
      interval_out$Group_kpa...10 #find the Group Wm values at this time, 2 less than gateway file
      func_data$Group_kpa_before_last_irr <- interval_out$Group_kpa...10
      
      #identify forced irr events and before WM turned on.
      func_data <- func_data %>%
        rowwise()%>%
        mutate(note = case_when(
          Group_kpa_before_last_irr == -999 ~ "Force Irr Event",
          as.POSIXlt(Group_last_irr) <= as.POSIXlt("2022-05-17 00:00:00 EST") ~ "Before stress",
          TRUE ~ "none"
        ))
      
      func_data <- func_data%>%
        select(node_id, Group_wm, Group_last_irr, Group_kpa_before_last_irr, note)
      
      
      # put into table graph object to display in plots..
      func_table <- plot(tableGrob(as.data.frame(func_data), rows = NULL, theme = ttheme_default(base_size=10)))
      
      return(func_table)
    }
    
    return(list(each_mux_channel, by_treatment_facet, by_treatment_no_facet, treatment_means_no_facet, group_means_no_facet, battery_levels(), irrigation_info()))  
    break
  } else {
  
my_data <- read.csv2(file = list.files(path = paste(file_path), pattern = "*.TXT", full.names = TRUE), header = FALSE, skip = 12) 
#skip=12
#start_line <- 1

ncolumns <- function(f){ #06_06_2022 witnessing error in reading gateway files
  data.start.line <- readLines(f, n=start_line)
 # print(data.start.line)
  dat_length <- length(strsplit(data.start.line, ",")[[start_line]])
  print(dat_length)
  
  end_line <- fpeek::peek_count_lines(f, with_eof = FALSE)
  data.end.line <- readLines(f, end_line)
  
  end_line_length <- length(strsplit(data.end.line, ",")[[end_line]])
 # print(end_line_length)
  if(end_line_length > dat_length){return(end_line_length)}else{return(dat_length)}
  
}

#ncolumns(list.files(path=paste("N:/Scientist/ABierer/Projects/Open Irr/preliminary test, January 2022/Raw_Data/Gateway/06_23_22"), pattern = ".TXT", full.names = TRUE))

length_header <- ncolumns(list.files(path = paste(file_path), pattern = "*.TXT", full.names = TRUE))
# length_header <- 77
str_split_fixed(my_data[,1], pattern=",", n=length_header)

#number_temp <- 6
#number_wm <- 16

gateway_info <- c("Gateway_ID", "Gateway_battV")

channel_info <- rep(c("mux_channel", "resistance_ohms", "matric_potential_kpa"), times=number_wm)
temp_info <- rep(c("temperature_sensor", "temperature_c"), times=number_temp)

channel_info <- paste(rep("mux_channel", times=number_wm), ".", 0:(number_wm-1))
channel_info <- gsub(" ", "", channel_info, fixed = TRUE)

resistance_info <- paste(rep("resistance_ohms", times=number_wm), ".", 0:(number_wm-1))
resistance_info <- gsub(" ", "", resistance_info, fixed = TRUE)

matric_info <- paste(rep("matric_potential_kpa", times=number_wm), ".", 0:(number_wm-1))
matric_info <- gsub(" ", "", matric_info, fixed = TRUE)

temp_sensor_info <- paste(rep("temperature_sensor", times=number_temp),".",0:(number_temp-1))
temp_sensor_info <- gsub(" ","", temp_sensor_info, fixed = TRUE)

temp_c_info <- paste(rep("temperature_c", times=number_temp),".",0:(number_temp-1))
temp_c_info <- gsub(" ","", temp_c_info, fixed = TRUE)

wm_info <- c(rbind(channel_info,resistance_info,matric_info))
temp_info <- c(rbind(temp_sensor_info,temp_c_info))

group_num <- c(paste(rep("Group_wm", times=4),".",0:3)) # times - 1
group_num <- gsub(" ","", group_num, fixed = TRUE)

group_kpa <- c(paste(rep("Group_kpa", times=4),".",0:3))
group_kpa <- gsub(" ","", group_kpa, fixed = TRUE)

group_last_irr <- c(paste(rep("Group_last_irr", times=4),".",0:3))
group_last_irr <- gsub(" ","", group_last_irr, fixed = TRUE)

wm_info <- c(rbind(channel_info,resistance_info,matric_info))
temp_info <- c(rbind(temp_sensor_info,temp_c_info))
group_info <- c(rbind(group_num,group_kpa,group_last_irr))

#WM_ON <- TRUE
if(is.null(WM_ON)){
  header_names <- c(gateway_info, "node_id", "node_voltage", "datetime", wm_info, temp_info)
} else {
  header_names <- c(gateway_info, "node_id", "node_voltage", "datetime", wm_info, group_info, temp_info)
}

my_data <- as.data.frame(str_split_fixed(my_data[,1], pattern = ",", n=length_header)) #only uses a fixed column length
colnames(my_data) <- header_names

my_data$record <- seq(1:dim(my_data)[1]) #specify a unique value for each row, here the sequence of row numbers

#test.data <- my_data
#my_data <- test.data

###Save wide format data for further processing 6/24/2022###
#write_csv(my_data, file = "N:/Scientist/ABierer/Projects/Open Irr/preliminary test, January 2022/Parsed_Data/gateway_wide.csv")

if(is.null(gateway_file)){
  my_data <- my_data %>%
    pivot_longer(names_to = "variable", values_to = "value", -c(record, node_id, node_voltage, datetime))%>%
    separate(variable, c("variable", "sensor"), sep = "[.]", fill = "right", extra = "merge", convert = TRUE)%>%
    pivot_wider(names_from = variable, values_from = value, values_fill = NA)

} else{
  my_data <- my_data %>%
    pivot_longer(names_to = "variable", values_to = "value", -c(record, node_id, node_voltage, datetime, Gateway_ID, Gateway_battV))%>%
    separate(variable, c("variable", "sensor"), sep = "[.]", fill = "right", extra = "merge", convert = TRUE)%>%
    pivot_wider(names_from = variable, values_from = value, values_fill = NA)
}
my_data$datetime <- as.POSIXct(my_data$datetime, tz="EST", format="%m/%d/%Y %H:%M:%S")
my_data$Group_last_irr <- as.POSIXct(my_data$Group_last_irr, tz="EST", format="%m/%d/%Y %H:%M:%S")
my_data$Group_kpa <- as.numeric(my_data$Group_kpa)
my_data$Group_wm <- as.factor(my_data$Group_wm)
my_data$matric_potential_kpa <- as.numeric(my_data$matric_potential_kpa)
my_data$mux_channel <- as.factor(my_data$mux_channel)
my_data$node_voltage <- as.numeric(my_data$node_voltage)
my_data$matric_potential_kpa[my_data$matric_potential_kpa > 0] <- 0 #Replace any positive matric potential values with 0
my_data$node_id <- as.numeric(as.character(my_data$node_id))
my_data <- my_data%>%
  filter(., node_id <= 105 & node_id >= 101)
my_data$node_id <- as.factor(my_data$node_id)

#view_window_start = "2022-05-27 01:00:00 EST"
#view_window_end =  "2022-06-06 08:30:00 EST"


if(is.null(view_window_start) | is.null(view_window_end)){
  #null_start <- (max(my_data$datetime) - lubridate::hours(48))
  #null_end <- max(my_data$datetime)
  #null_interval <- lubridate::interval(start = null_start, end = null_end, tz="EST) # define time window
  my_data <- my_data #[my_data$datetime %within% null_interval,] # return values within the time window
} else {
  selected_interval <- lubridate::interval(start = view_window_start, end = view_window_end, tz="EST")
  my_data <- my_data[my_data$datetime %within% selected_interval,]
}
my_data <- my_data %>%
  rowwise()%>% # is this necessary? It slows down alot....
  mutate(treatment = case_when(
    ((mux_channel == 4 | mux_channel == 5 | mux_channel == 11) &  (node_id == 101))  ~  "Control",
    ((mux_channel == 6 | mux_channel == 8 | mux_channel == 9) & (node_id == 102)) ~ "Control",
    ((mux_channel == 3 | mux_channel == 7 | mux_channel == 9) & (node_id == 103)) ~ "Control",
    ((mux_channel == 7 | mux_channel == 8 | mux_channel == 11) & (node_id == 104)) ~ "Control",
    ((mux_channel == 4 | mux_channel == 7 | mux_channel == 10) & (node_id == 105)) ~ "Control",
    
    ((mux_channel == 6 | mux_channel == 8 | mux_channel == 12) & (node_id == 101)) ~  "-50 kPa",
    ((mux_channel == 4 | mux_channel == 7 | mux_channel == 11) & (node_id == 102)) ~ "-50 kPa",
    ((mux_channel == 1 | mux_channel == 2 | mux_channel == 6) & (node_id == 103)) ~ "-50 kPa",
    ((mux_channel == 4 | mux_channel == 5 | mux_channel == 10) & (node_id == 104)) ~ "-50 kPa",
    ((mux_channel == 2 | mux_channel == 8 | mux_channel == 11) & (node_id == 105))  ~ "-50 kPa",
    
    ((mux_channel == 1 | mux_channel == 7 | mux_channel == 9) & (node_id == 101)) ~ "-100 kPa",
    ((mux_channel == 1 | mux_channel == 10 | mux_channel == 12) & (node_id == 102)) ~ "-100 kPa",
    ((mux_channel == 5 | mux_channel == 8 | mux_channel == 11) & (node_id == 103)) ~ "-100 kPa",
    ((mux_channel == 3 | mux_channel == 6 | mux_channel == 12) & (node_id == 104))  ~ "-100 kPa",
    ((mux_channel == 1 | mux_channel == 6 | mux_channel == 9) & (node_id == 105)) ~ "-100 kPa",
    
    ((mux_channel == 2 | mux_channel == 3 | mux_channel == 10) & (node_id == 101)) ~ "-200 kPa",
    ((mux_channel == 2 | mux_channel == 3 | mux_channel == 5) & (node_id == 102)) ~ "-200 kPa",
    ((mux_channel == 4 | mux_channel == 10 | mux_channel == 12) & (node_id == 103)) ~ "-200 kPa",
    ((mux_channel == 1 | mux_channel == 2 | mux_channel == 9) & (node_id == 104)) ~ "-200 kPa",
    ((mux_channel == 3 | mux_channel == 5 | mux_channel == 12) & (node_id == 105)) ~ "-200 kPa",
    
    TRUE ~ "Error"
  ))
unfiltered_data <-  my_data #for grabbing from unaligned group_wm data.

###Write long data to file for use in later analysis 6/24/2022###
#write_csv(my_data, file = "N:/Scientist/ABierer/Projects/Open Irr/preliminary test, January 2022/Parsed_Data/gateway_long.csv")

######below
unfiltered_data <- unfiltered_data %>%
  filter(., Group_wm == "G1" || Group_wm == "G2" || Group_wm =="G3" || Group_wm =="G4") # removing partial transmissions from the gateway until its fixed in program...
unfiltered_data <- unfiltered_data%>%
  filter(., Group_kpa >= -255 && Group_kpa <= 0) #again removing partial transmissions.

#unfiltered_data <- unfiltered_data%>%
#  filter(treatment != "Error")

my_data <- my_data%>%
  filter(treatment != "Error")

my_data <- my_data%>%
  filter(matric_potential_kpa <= 0)

my_data$treatment <- as.factor(my_data$treatment)


# Make Graphs of interest  
each_mux_channel  <- ggplot(my_data)+
  geom_line(mapping = aes(x=datetime, y=matric_potential_kpa, group=mux_channel, color=mux_channel))+
  scale_x_datetime()+
  scale_y_continuous(limits = c(-250,0))+
  scale_color_igv()+
  facet_wrap("node_id")

by_treatment_facet <-  ggplot(my_data)+
  geom_point(mapping = aes(x=datetime, y=matric_potential_kpa, group=treatment, color=treatment))+
  scale_x_datetime()+
  scale_y_continuous(limits = c(-250,0))+
  scale_color_npg()+
  facet_wrap("node_id")

by_treatment_no_facet <-  ggplot(my_data)+ # too busy to show much..
  geom_point(mapping = aes(x=datetime, y=matric_potential_kpa, group=treatment, color=treatment))+
  scale_x_datetime()+
  scale_y_continuous(limits = c(-250,0))+
  scale_color_npg()

fun0 <- function(x) 0

if(is.null(view_window_start) | is.null(view_window_end)){
  graph_label_position <- as.POSIXct(max(my_data$datetime), tz="EST") 
} else {
  graph_label_position <-as.POSIXct(view_window_end, tz="EST") 
}

treatment_means_no_facet_data_table <- function(){
  func_data <- my_data %>%
    group_by(treatment)%>%
    summarize(treatment_mean = mean(matric_potential_kpa),
              treatment_stdev = sd(matric_potential_kpa))
  
  func_table <- tableGrob(as.data.frame(func_data), rows = NULL, theme = ttheme_default(base_size=10))
  return(func_table)
}

treatment_means_no_facet <- ggplot(my_data)+
  geom_point(mapping = aes(x=datetime, y=matric_potential_kpa, group=treatment, color=treatment), stat= "summary", fun="mean")+
  #stat_summary(fun = 'mean', geom = 'text', aes(x = as.POSIXct(graph_label_position), y= matric_potential_kpa, label=paste("mean:",signif(..y..,4)), group=treatment))+
  stat_summary(fun = 'mean', geom = 'text', aes(x = as.POSIXct(graph_label_position), y= matric_potential_kpa, label=paste("mean:",signif(..y..,4)), group=treatment, color=treatment), position = position_nudge(x=12*60*60), show.legend=FALSE)+ # make the call to x position the end date window
  stat_summary(fun = 'mean', fun.min = 'sd', geom = 'text', aes(x = as.POSIXct(graph_label_position), y=matric_potential_kpa, label=paste("sd:", signif(..ymin..,4)), group=treatment, color=treatment), position = position_nudge(x=24*60*60), show.legend=FALSE)+ #x=as.integer(treatment)
  annotation_custom(treatment_means_no_facet_data_table(), xmin = graph_label_position - (24*60*60), xmax = graph_label_position - (60*60), ymin = -100, ymax = -75)+
  scale_x_datetime()+
  scale_y_continuous(limits = c(-250,0))+
  scale_color_npg()

#Do treatment means except by group means where the mean is buffered against outliers by the system?

group_means_no_facet_data_table <- function(){
  func_data <- unfiltered_data%>%
    group_by(Group_wm)%>%
    na.omit()%>%
    summarize(Group_wm_mean = mean(Group_kpa),
              Group_wm_stdev = sd(Group_kpa),
              Group_wm_N = length(Group_kpa))
  func_table <- tableGrob(as.data.frame(func_data), rows = NULL, theme = ttheme_default(base_size = 10))
  return(func_table)
}

group_means_no_facet <- ggplot(unfiltered_data)+
  geom_point(mapping = aes(x=datetime, y=Group_kpa, group=Group_wm, color=Group_wm), stat = "summary", fun="mean")+
  stat_summary(fun = 'mean', geom = 'text', aes(x = as.POSIXct(graph_label_position), y= Group_kpa, label=paste("mean:",signif(..y..,4)), group=Group_wm, color=Group_wm), position = position_nudge(x=12*60*60), show.legend=FALSE)+ # make the call to x position the end date window
  stat_summary(fun = 'mean', fun.min = 'sd', geom = 'text', aes(x = as.POSIXct(graph_label_position), y=Group_kpa, label=paste("sd:", signif(..ymin..,4)), group=Group_wm, color=Group_wm), position = position_nudge(x=24*60*60), show.legend=FALSE)+ #x=as.integer(treatment)
  annotation_custom(group_means_no_facet_data_table(), xmin = graph_label_position - (24*60*60), xmax = graph_label_position - (60*60), ymin = -100, ymax = -75)+
  scale_x_datetime()+
  scale_y_continuous(limits = c(-250,0))+
  scale_color_npg()

#Vizualize Battery Levels
battery_levels <- function(){
  
  func_data <- my_data%>%
    group_by(node_id)%>%
    filter(datetime == max(datetime))%>%
    summarise(node_voltage = min(node_voltage)) # minimum voltage at most recent datetime for each node
  
  
  battery_func_plot <- ggplot(func_data)+
    geom_col(mapping = aes(x=node_id, y=node_voltage, group=node_id, fill=node_id))+
    geom_abline(intercept = 6.5, slope = 0, linetype=2, color="red")+
    annotate("text", x=1.75, y=6.0, label="Change battery pack below this line.")+
    scale_color_npg()
  
  
  return(battery_func_plot)
}


#max(my_data$datetime) - min(my_data$datetime)

#Count & display number of irrigation events and the last irrigation event to occur for each group/node
#As of 2022-05-26 this has been amended since the last irrigation event times are not equal to datetime
#in the data string. See explanation below for additional details.
irrigation_info <- function(){
  func_data <- unfiltered_data %>%
    group_by(node_id, Group_wm)%>%
    filter(Group_wm == "G1" || Group_wm =="G2"||Group_wm=="G3"||Group_wm=="G4")%>%
    slice_max(Group_last_irr, n = 1, with_ties = FALSE) # finds the max, here latest time, of Group_last_irr in the dataframe
  
  interval_test <- outer(func_data$Group_last_irr, unfiltered_data$datetime, "-") # get time difference between (x,y)
  interval_test[interval_test < 0] <- NA # remove where datetime is after Group_last_irr, leaving values before Group_last_irr 
  ind <- as.numeric(apply(interval_test, 1, function(i) which.min(i))) # find index of minimum, where the time between / interval is the smallest
  interval_out <- cbind(func_data, unfiltered_data[ind,]) #output data ****06/06/20222 error here
  #interval_out
  match(as.POSIXlt(interval_out$datetime...5), as.POSIXlt(unfiltered_data$datetime)) #not sure if this
  #match(as.POSIXlt(interval_out$datetime...22), as.POSIXlt(unfiltered_data$datetime)) #or this
  interval_out$Group_kpa...12 #find the Group Wm values at this time
  func_data$Group_kpa_before_last_irr <- interval_out$Group_kpa...12
  
  #identify forced irr events and before WM turned on.
  func_data <- func_data %>%
    rowwise()%>%
    mutate(note = case_when(
      Group_kpa_before_last_irr == -999 ~ "Force Irr Event",
      as.POSIXlt(Group_last_irr) <= as.POSIXlt("2022-05-17 00:00:00 EST") ~ "Before stress",
      TRUE ~ "none"
    ))
  
  func_data <- func_data%>%
    select(node_id, Group_wm, Group_last_irr, Group_kpa_before_last_irr, note)
    
  
  # put into table graph object to display in plots..
  func_table <- plot(tableGrob(as.data.frame(func_data), rows = NULL, theme = ttheme_default(base_size=10)))
  
  return(func_table)
}

##I think I know what is wrong.... The time of the last irr event is programmed to be different from
##the datetime. This is because the datetime is determined at the start of the run routine and the time of
##last irrigation event is triggered and saved in that particular subroutine. Therefore, it should NOT be
##expected that these two datetimes lineup.
##A solution could be given by finding the datetime immediately preceding that irrigation time for each node_id and Group_wm
##combination.This should allow retrieval of the true group_wm value observed before the irrigation event was triggered
  
# pony <- unfiltered_data%>%
#   group_by(node_id, Group_wm)%>%
#   filter(Group_wm == "G1" || Group_wm =="G2"||Group_wm=="G3"||Group_wm=="G4")%>%
#   slice_max(Group_last_irr, n = 1, with_ties = FALSE) # finds the max, here latest time, of Group_last_irr in the dataframe
# 
#   #using library(zoo)
#   #interval_test <- findInterval(index(pony$Group_last_irr), index(unfiltered_data$datetime)) # find the interval between max Group_last_irr determined above and datetime column in dataframe, i.e., where time between is positive
#   #interval_test <- ifelse(interval_test == 0, NA, interval_test) # if interval is 0, 
#   #pony$interval <- index(unfiltered_data$datetime)[interval_test]
# 
#   interval_test <- outer(pony$Group_last_irr, unfiltered_data$datetime, "-") # get time difference between (x,y)
#   interval_test[interval_test < 0] <- NA # remove where datetime is after Group_last_irr, leaving values before Group_last_irr 
#   ind <- apply(interval_test, 1, function(i) which.min(i)) # find index of minimum, where the time between / interval is the smallest
#   interval_out <- cbind(pony, unfiltered_data[ind,]) #output data
#   #interval_out
#   match(as.POSIXlt(interval_out$datetime...5), as.POSIXlt(unfiltered_data$datetime)) #not sure if this
#   #match(as.POSIXlt(interval_out$datetime...22), as.POSIXlt(unfiltered_data$datetime)) #or this
#   interval_out$Group_kpa...12 #find the Group Wm values at this time
#   pony$Group_kpa_before_last_irr <- interval_out$Group_kpa...12
#   
#   #identify forced irr events and before WM turned on.
#   pony <- pony %>%
#     rowwise()%>%
#     mutate(note = case_when(
#     Group_kpa_before_last_irr == -999 ~ "Force Irr Event",
#     as.POSIXlt(Group_last_irr) <= as.POSIXlt("2022-05-17 00:00:00 EST") ~ "Before stress",
#     TRUE ~ "none"
#   ))
# 
#   pony%>%
#     select(node_id, Group_wm, Group_last_irr, Group_kpa_before_last_irr, note)

###  
  

return(list(each_mux_channel, by_treatment_facet, by_treatment_no_facet, treatment_means_no_facet, group_means_no_facet, battery_levels(), irrigation_info(), my_data))
}
}


#Example
vizualize_sensors(file_path = "N:/Scientist/ABierer/Projects/Open Irr/Raw_Data/Gateway/06_13_22/",
                  gateway_file = TRUE,
                  start_line = 1,
                  number_wm = 16,
                  number_temp = 6,
                  view_window_start = "2022-06-06 00:00:00 EST",
                  view_window_end =  "2022-06-13 24:00:00 EST",
                  WM_ON = TRUE)

  
