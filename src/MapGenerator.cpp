#include "MapGenerator.h"


void MapGenerator::Refresh()
{

    wtb.setIdentity();
    mTfBr.sendTransform(tf::StampedTransform(wtb,ros::Time::now(), "/CamLoc/World", "/CamLoc/Camera"));

    read_velodyne(str_path_+"sequences/00/velodyne/",data_count);


    if(data_count>0){
        EST_pose = EST_pose*poses[data_count-1].inverse()*poses[data_count];
        pcl::transformPointCloud (*velo_cloud, *velo_cloud, EST_pose*cTv);
        //ICP between merged_cloud and velo_cloud
        VCICP(merged_cloud, velo_cloud);
    }
    else{
        EST_pose = poses[data_count];
        cout<<EST_pose<<endl;
        pcl::transformPointCloud (*velo_cloud, *velo_cloud, EST_pose*cTv);
    }    
    (*merged_cloud) += (*velo_cloud); //merge point clouds
    //publish map and poses
    MapPub.PublishMap(merged_cloud,1);
    MapPub.PublishPose(poses[data_count],1);
    MapPub.PublishPose(EST_pose,3);
    cout<<"Map has been updated"<<endl;
 
    data_count++;

//    if(data_count%3 == 0){   
        //save velodyne point clouds
        std::string fname;
        char str[7];
        snprintf (str, 7, "%06d", data_count);
        fname = fname+str+".ply";
        pcl::io::savePLYFileASCII (fname.c_str(), *merged_cloud);
        merged_cloud->clear();
        (*merged_cloud) = (*velo_cloud);
//    }

    //write poses
    ofstream poses_file("poses.txt", std::ios::app);
    if (poses_file.is_open())poses_file << EST_pose.block<3,4>(0,0) << '\n';
    poses_file.close();

//    float *data = (float*)malloc(12*sizeof(float));
//    for (int32_t i=0; i<12; i++){
//        int u = i/4;
//        int v = i%4;
//        float *px;
//        px = data+i;
//        *px = EST_pose(u,v);
//    }
//    poses_file = fopen ("poses.txt","wb");
//    fwrite (data , sizeof(float), 12, poses_file);
//    fclose (poses_file);
    
}

void MapGenerator::read_velodyne(std::string fname, int idx)
{
  // allocate 4 MB buffer (only ~130*4*4 KB are needed)
  int32_t num = 1000000;
  float *data = (float*)malloc(num*sizeof(float));

  // pointers
  float *px = data+0;
  float *py = data+1;
  float *pz = data+2;
  //float *pr = data+3;

  // load point cloud
  FILE *stream;
  char str[7];
  snprintf (str, 7, "%06d", idx);
  fname = fname+str+".bin";
  cout<<fname<<endl;
  stream = fopen (fname.c_str(),"rb");
  velo_cloud->clear();
  num = fread(data,sizeof(float),num,stream)/4;
  for (int32_t i=0; i<num; i++) {
    velo_cloud->points.push_back(pcl::PointXYZ(*px,*py,*pz));
    px+=4; py+=4; pz+=4; //pr+=4;
  }
  fclose(stream);

}

void MapGenerator::read_ctv(std::string fname)
{
    FILE *fl;
    char s[50];
    int count = 0;

    if ((fl = fopen(fname.c_str(), "r")) != NULL){
        while(fscanf(fl, "%s", &s) != EOF){
            if(count >0){
                int i = (count-1)/4;
                int j = (count-1)%4;
                cTv(i,j) = atof(s);
                count++;
            }
            if(!strcmp(s,"Tr:")){
                count = 1;
            }
        }
    }
    cout<<cTv<<endl;
}

void MapGenerator::read_poses(std::string fname)
{
    FILE *fl;
    char s[50];
    int count = 0;
    poses.reserve(4541);
    Matrix4f tmp = Matrix4f::Identity();

    if ((fl = fopen(fname.c_str(), "r")) != NULL){
        while(fscanf(fl, "%s", &s) != EOF){
            int i = count/4;
            int j = count%4;
            tmp(i,j) = atof(s);
            count++;
            if(count == 12){
                count = 0;
                poses.push_back(tmp);
                tmp = Matrix4f::Identity();
                
            }
        }
    }
}

void MapGenerator::VCICP(pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_points,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& src_points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    vector<int> sampled_indices;
    for (size_t i = 0; i < src_points->points.size(); ++i)
    {
        if (i%10==0)sampled_indices.push_back(i);
    } 
    pcl::copyPointCloud(*src_points, sampled_indices, *src_filtered); 

    cout<<"Before ICP: "<<tgt_points->points.size()<<", "<<src_points->points.size()<<endl;
    
    //create octree
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> LPDoctree(resolution);
    LPDoctree.setInputCloud (tgt_points);
    LPDoctree.addPointsFromInputCloud ();


    Matrix4f S_sum=Matrix4f::Identity();
    Matrix4f S=Matrix4f::Identity();
//    S(2,3) -= 0.1f;
//    S(0,3) += 0.01f;
//    pcl::transformPointCloud (*src_filtered, *src_filtered, S);//transform c_cam
//    S_sum = S*S_sum;
//    S=Matrix4f::Identity(); 
    //ICP iterations
    for (size_t i = 0; i < max_iter; i++)
    {
        //cout<<"ICP iteration: "<<i<<endl;
        CorrespondenceUpdate(tgt_points,src_filtered,i,LPDoctree);

        S = Optimization(tgt_points,src_filtered,10);
        pcl::transformPointCloud (*src_filtered, *src_filtered, S);//transform c_cam
        S_sum = S*S_sum;
        cout<<S<<endl;
        if((abs(S(0,3))+abs(S(1,3))+abs(S(2,3)))<0.003)break;
    }
    pcl::transformPointCloud (*src_points, *velo_cloud, S_sum);//transform c_cam
    EST_pose = EST_pose*S_sum; 
}

void MapGenerator::CorrespondenceUpdate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& v_cloud,
                                           const pcl::PointCloud<pcl::PointXYZ>::Ptr& c_cloud,
                                           const int iter,
                                           pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> LPDoctree)
{
    cout<<"Correspondence Update"<<endl;
    iv_corr.i_idx.clear();
    iv_corr.v_idx.clear();

    //initial correspondence (NN search)
    for (size_t i = 0; i < c_cloud->points.size(); ++i)
    {
        int res_idx;
        float res_dist;
        LPDoctree.approxNearestSearch (*c_cloud, i, res_idx, res_dist);
        if(res_dist<-(tmax-tmin)*iter/max_iter+tmax)
        {
            iv_corr.i_idx.push_back(i);        
            iv_corr.v_idx.push_back(res_idx);
            iv_corr.info.push_back(100);
        }
    }
}

Matrix4f MapGenerator::Optimization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& v_cloud,
                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& c_cloud,
                                       float th2)
{
    //g2o optimization 
    cout<<"g2o Optimization"<<endl;
    const float deltaHuber = sqrt(th2);
    int max_size=iv_corr.i_idx.size();
    cout<<"test2: "<<iv_corr.i_idx.size()<<endl;

    //solver initialization
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);


    // SET SIMILARITY VERTEX
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();
    vSim3->_fix_scale= true;
    Matrix3d R = Matrix3d::Identity();
    Vector3d t(0,0,0);
    const double s = 1;
    g2o::Sim3 g2oS_init(R,t,s);
    vSim3->setEstimate(g2oS_init);
    vSim3->setId(0);
    vSim3->setFixed(false);
//    vSim3->_principle_point1[0] = K(2,0);
//    vSim3->_principle_point1[1] = K(2,1);
//    vSim3->_focal_length1[0] = K(0,0);
//    vSim3->_focal_length1[1] = K(1,1);
//    vSim3->_principle_point2[0] = K(2,0);
//    vSim3->_principle_point2[1] = K(2,1);
//    vSim3->_focal_length2[0] = K(0,0);
//    vSim3->_focal_length2[1] = K(1,1);
    optimizer.addVertex(vSim3);

    //Set map point vertices
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;
    vnIndexEdge.reserve(2*max_size);
    vpEdges12.reserve(2*max_size);
    vpEdges21.reserve(2*max_size);


    for (size_t i = 0; i < max_size; i++)
    {

        int id1 = 2*i+1;
        int id2 = 2*(i+1);

        // SET PointXYZ VERTEX
        g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
        Matrix<double,3,1> pt1 (c_cloud->points[iv_corr.i_idx[i]].x, c_cloud->points[iv_corr.i_idx[i]].y, c_cloud->points[iv_corr.i_idx[i]].z);
        vPoint1->setEstimate(pt1);
        vPoint1->setId(id1);
        vPoint1->setFixed(true);
        optimizer.addVertex(vPoint1);
        //cout<<"pt1: "<<vSim3->cam_map1(g2o::project(pt1))<<endl;

        g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
        Matrix<double,3,1> pt2 (v_cloud->points[iv_corr.v_idx[i]].x, v_cloud->points[iv_corr.v_idx[i]].y, v_cloud->points[iv_corr.v_idx[i]].z);
        vPoint2->setEstimate(pt2);
        vPoint2->setId(id2);
        vPoint2->setFixed(true);
        optimizer.addVertex(vPoint2);


        // Set Edge
        Matrix<double,3,1> obs1;
        obs1 = pt2;//vSim3->cam_map1(g2o::project(pt2));
        //cout<<"pt2: "<<obs1<<endl;

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        e12->setInformation(Matrix3d::Identity()*iv_corr.info[i]);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // SET EDGE c_image = project(S12.inverse()*v_cloud)
        Matrix<double,3,1> obs2;
        obs2 = pt1;//vSim3->cam_map1(g2o::project(pt1));

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();
        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        e21->setInformation(Matrix3d::Identity()*iv_corr.info[i]);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);

    }
    optimizer.initializeOptimization();
    optimizer.optimize(5);
    
    // Check inliers
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=NULL;
            vpEdges21[i]=NULL;
            nBad++;
        }
    }

    cout<<"Number of nBad is: "<<nBad<<endl;
    int nMoreIterations;
    if(nBad>0){
        nMoreIterations=5;
        if((max_size/nBad)<3)//((max_size-nBad*2)<50)//((max_size/nBad)<2.1)//(nBad>500)//
        {
            cout<<"Number of nBad is large"<<endl;
            nMoreIterations=-1;
        }
    }
    else
        nMoreIterations=5;



    if(nMoreIterations>0){
        cout<<nMoreIterations<<" more optimizations"<<endl;
        // Optimize again only with inliers
        optimizer.initializeOptimization();
        optimizer.optimize(nMoreIterations);
        // Recover optimized Sim3
        g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
        g2o::Sim3 g2oS12= vSim3_recov->estimate();      
        return Sim3toMat(g2oS12);
    }
    else{
//        g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
//        g2o::Sim3 g2oS12= vSim3_recov->estimate();      
//        return Sim3toMat(g2oS12);
        return Matrix4f::Identity();
    }
    
}
