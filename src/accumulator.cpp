class Accumulator {

   private:
    int pcl_type;
    sensor_msgs::PointCloud2 accum_pcl;
    int last_timestamp_lidar_part;
    int parts_received;

   public:

    std::string topic;

    Accumulator(std::string topic, int type) {
        this->topic = topic;
        this->pcl_type = type;
        parts_received = 0;
    }

    void receive(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        this->last_timestamp_lidar_part = msg->header.timestamp;
        this->accum_pcl += *msg;
        ++parts_received;

        this->prefilter();
    }

    void prefilter() { return; }
    void postfilter() { return; }

    sensor_msgs::PointCloud2 get() {
        this->postfilter();
        sensor_msgs::PointCloud2 temp = accum_pcl;
        accum_pcl.clear();
        parts_received = 0;
        return temp;
    }

    void push() {
        mtx_buffer.lock();

        const sensor_msgs::PointCloud2::ConstPtr msg = *accum_pcl;

        if (msg->header.stamp.toSec() < last_timestamp_lidar)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer.clear();
        }

        PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        
        lidar_buffer.push_back(ptr);
        
        time_buffer.push_back(msg->header.stamp.toSec());
        last_timestamp_lidar = msg->header.stamp.toSec();
        
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }
};