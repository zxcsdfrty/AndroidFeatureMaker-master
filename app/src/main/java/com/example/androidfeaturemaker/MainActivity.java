package com.example.androidfeaturemaker;
import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;
import android.graphics.drawable.BitmapDrawable;
import android.icu.util.TimeZone;
import android.os.Environment;
import android.os.Handler;
import android.os.Message;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.FpsMeter;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FastFeatureDetector;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;


import java.io.DataInputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.text.DecimalFormat;

import static org.opencv.core.Core.add;
import static org.opencv.core.Core.eigen;
import static org.opencv.core.Core.gemm;
import static org.opencv.core.Core.log;
import static org.opencv.core.Core.pow;
import static org.opencv.core.Core.sqrt;
import static org.opencv.features2d.DescriptorMatcher.BRUTEFORCE_HAMMING;
import static org.opencv.features2d.Features2d.drawKeypoints;
public class MainActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2{

    protected static final float FLIP_DISTANCE = 150;

    private ExecutorService mThreadPool;
    //網路串流
    private Socket socket;
    OutputStream outputStream;
    //紀錄登入序號
    int playerList;
    //紀錄玩家資訊
    Point3 unityPosition;
    int x = 0, y = 55, z = 0;
    int viewX = 90, viewY = 0, viewZ = 0;
    int moveX=0, moveY=0;
    String st;
    //test
    private Button btnSend;
    ImageView imgView;
    private Handler mMainHandler;
    Bitmap bmp;
    Bitmap bmp32;
    //速度優化
    int buffer = 0;
    private Lock lock = new ReentrantLock();
    DecimalFormat decimalFormat = new DecimalFormat("##.000");

    //解析度在JavaCameraView調
    JavaCameraView javaCameraView;
    private static String Tag = "MainActivity";
    GestureDetector mGesture;
    FeatureDetector featureDetector = FeatureDetector.create(FeatureDetector.AKAZE);//opencv不支持SIFT、SURF检测方法
    DescriptorExtractor descriptorExtractor = DescriptorExtractor.create(DescriptorExtractor.AKAZE);
    //match descriptor vectors
    DescriptorMatcher matcher = DescriptorMatcher.create(BRUTEFORCE_HAMMING);
    MatOfDMatch matches = new MatOfDMatch();
    MatOfKeyPoint keyPoint_train =new MatOfKeyPoint();
    MatOfKeyPoint keyPoint_test =new MatOfKeyPoint();
    Mat descriptor1 =new Mat();
    Mat descriptor2 =new Mat();
    Mat mRgba = new Mat();
    Mat mGray = new Mat();
    //Mat oldpaste=new Mat();
    Mat paste = new Mat();
    Mat oldmaker= new Mat();
    Mat maker = new Mat();
    Mat makerGray = new Mat();
    //Mat obj_pixel = new Mat((int)size.height,(int)size.width,CvType.CV_32FC2);
    //Mat scene_pixel = new Mat((int)size.height,(int)size.width,CvType.CV_32FC2);
    Mat cameraMatrix=new Mat(3,3,CvType.CV_32F);//CV_32F：32-bit ﬂoating-point numbers
    MatOfDouble distCoeffs=new MatOfDouble();
    Mat Tvec=new Mat();
    Mat Rvec=new Mat();

    BaseLoaderCallback baseLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
            switch (status) {
                case BaseLoaderCallback.SUCCESS: {
                    javaCameraView.enableView();
                    break;
                }
                default: {
                    super.onManagerConnected(status);
                    break;
                }
            }
        }
    };

    static {
        if (OpenCVLoader.initDebug()) {
            Log.i(Tag, "opencv loaded");
        } else {
            Log.i(Tag, "opencv not loaded");
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        btnSend = (Button) findViewById(R.id.Begin);
        //imgView = (ImageView) findViewById(R.id.image);
        //imgView.setImageBitmap(bmp);

        Button calibration = (Button)findViewById(R.id.Calibration);
        calibration.setOnClickListener(new Button.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent=new Intent();
                intent.setClass(MainActivity.this,Calibration.class);
                startActivity(intent);
            }
        });

        //相機內部參數
        //cameraMatrix.put(0,0,608.33742247);
        cameraMatrix.put(0,0,550.33742247);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2,360.62381387);
        cameraMatrix.put(1,0,0);
        //cameraMatrix.put(1,1,607.41692009);
        cameraMatrix.put(1,1,550.41692009);
        cameraMatrix.put(1,2,240.04093524);
        cameraMatrix.put(2,0,0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);

        //distCoeffs
        distCoeffs.put(0,0,0.114998524);
        distCoeffs.put(0,1,-0.0399730218);
        distCoeffs.put(0,2,0.00108110572);
        distCoeffs.put(0,3,-0.000319788278);
        distCoeffs.put(0,4,-0.699416596);

        bmp = BitmapFactory.decodeResource(getResources(), R.drawable.p1);
        bmp32 = bmp.copy(Bitmap.Config.ARGB_8888, true);
        Utils.bitmapToMat(bmp32, oldmaker);
        //Imgproc.resize(oldmaker, maker,new Size(oldmaker.cols()/4, oldmaker.rows()/4));//调用Imgproc的Resize方法，进行图片缩放
        Imgproc.pyrDown(oldmaker, maker, new Size(oldmaker.cols()/2, oldmaker.rows()/2));

        /*
        bmp = BitmapFactory.decodeResource(getResources(), R.drawable.p2);
        bmp32 = bmp.copy(Bitmap.Config.ARGB_8888, true);
        Utils.bitmapToMat(bmp32, oldpaste);
        Imgproc.resize(oldpaste, paste,size);// 將圖片大小設為跟maker一樣
        */

        //將maker轉成灰階
        Imgproc.cvtColor(maker, makerGray, Imgproc.COLOR_RGB2GRAY);
        //偵測MAKER的keypoint and descriptor
        featureDetector.detect(makerGray,keyPoint_train);
        descriptorExtractor.compute(makerGray,keyPoint_train,descriptor1);

        //If authorisation not granted for camera
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED)
            //ask for authorisation
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA}, 50);
        javaCameraView = (JavaCameraView) findViewById(R.id.java_camera_view);
        javaCameraView.setVisibility(SurfaceView.VISIBLE);
        javaCameraView.setCvCameraViewListener(this);

        mGesture = new GestureDetector(this,new GestureDetector.SimpleOnGestureListener()
        {
            @Override
            public boolean onSingleTapUp(MotionEvent e) {
                // TODO Auto-generated method stub
                return false;
            }

            @Override
            public void onShowPress(MotionEvent e) {
                // TODO Auto-generated method stub

            }

            @Override
            public boolean onScroll(MotionEvent e1, MotionEvent e2, float distanceX, float distanceY) {
                // TODO Auto-generated method stub
                return false;
            }

            @Override
            public void onLongPress(MotionEvent e) {
                // TODO Auto-generated method stub

            }

            @Override
            public boolean onFling(MotionEvent e1, MotionEvent e2, float velocityX, float velocityY) {
                if (e1.getX() - e2.getX() > FLIP_DISTANCE) {
                    Log.i(Tag, "向左滑...");
                    moveX = -1;
                    Log.i(Tag, e1.getX() + " " + e1.getY());
                    Log.i(Tag, e2.getX() + " " + e2.getY());
                    return true;
                }
                if (e2.getX() - e1.getX() > FLIP_DISTANCE) {
                    Log.i(Tag, "向右滑...");
                    moveX = 1;
                    Log.i(Tag, e1.getX() + " " + e1.getY());
                    Log.i(Tag, e2.getX() + " " + e2.getY());
                    return true;
                }
                if (e1.getY() - e2.getY() > FLIP_DISTANCE) {
                    Log.i(Tag, "向上滑...");
                    moveY = 1;
                    Log.i(Tag, e1.getX() + " " + e1.getY());
                    Log.i(Tag, e2.getX() + " " + e2.getY());
                    return true;
                }
                if (e2.getY() - e1.getY() > FLIP_DISTANCE) {
                    Log.i(Tag, "向下滑...");
                    moveY = -1;
                    Log.i(Tag, e1.getX() + " " + e1.getY());
                    Log.i(Tag, e2.getX() + " " + e2.getY());
                    return true;
                }
                return false;
            }

            @Override
            public boolean onDown(MotionEvent e) {
                // TODO Auto-generated method stub
                return false;
            }
        });

        /*-------------------------------------------------------------------------------------------*/
        // 初始化線程
        mThreadPool = Executors.newCachedThreadPool();
        /*mMainHandler = new Handler() {
            @Override
            public void handleMessage(Message msg) {
                imgView.setImageBitmap(bmp);
                switch (msg.what) {
                    case 0:
                        //receive_message.setText(input);
                        break;
                }
            }

        };*/
        mThreadPool.execute(new Runnable() {
            @Override
            public void run() {
                try {
                    // 創建socket IP port
                    socket = new Socket("140.121.197.164", 80);
                    // 判斷是否連接成功
                    //System.out.println(socket.isConnected());
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException ex) {
                        Thread.currentThread().interrupt();
                    }
                    // record player's list
                    InputStream in = socket.getInputStream();
                    playerList = in.read();
                    Log.i("playerlist", "playerlist : "+playerList);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        return mGesture.onTouchEvent(event);
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (javaCameraView != null) {
            javaCameraView.disableView();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (javaCameraView != null) {
            javaCameraView.disableView();

        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (OpenCVLoader.initDebug()) {
            Log.i(Tag, "opencv loaded");
            baseLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);

        } else {
            Log.i(Tag, "opencv not loaded");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_4_0, this, baseLoaderCallback);
        }
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);//CV_(位元數)+(資料型態)+(Channel數)
        mGray = new Mat(height, width, CvType.CV_8UC1);
    }

    @Override
    public void onCameraViewStopped() {
        mRgba.release();
        mGray.release();
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {


        //the camera view size & orientation issue can be fix in
        //CameraBridgeViewBase.java in opencv library
        //in the function "deliverAndDrawFrame"
        mRgba = inputFrame.rgba();
        mGray=inputFrame.gray();

        //偵測CAMERA的keypoint and descriptor*/
        featureDetector.detect(mGray,keyPoint_test);
        descriptorExtractor.compute(mGray,keyPoint_test,descriptor2);

        matcher.match(descriptor1, descriptor2, matches);
        List<DMatch> matchesList = matches.toList();

        //若沒有任何點match直接return,可以防止當matchesList沒有任何東西時OutOfBoundary的情況
        if(matchesList.isEmpty())
            return mRgba;

        Double max_dist = 0.0;
        Double min_dist = 100.0;
        Log.i("descriptor"," row: "+descriptor1.rows()+" col: "+descriptor1.cols());
        for(int i = 0; i < descriptor1.rows(); i++){
                Double dist = (double) matchesList.get(i).distance;
                if (dist < min_dist) min_dist = dist;
                if (dist > max_dist) max_dist = dist;
        }

        Log.i("distance","min: "+min_dist+" max: "+max_dist);
        if(min_dist > 30 )
            return mRgba;

        LinkedList<DMatch> good_matches = new LinkedList<DMatch>();
        MatOfDMatch gm = new MatOfDMatch();
        //對匹配結果進行篩選
        for(int i = 0; i < descriptor1.rows(); i++){
            if(matchesList.get(i).distance <= 3*min_dist){
                good_matches.addLast(matchesList.get(i));
            }
        }

        Log.i("goodMatcheSize"," "+good_matches.size());
        if(good_matches.size() < 4 )
            return mRgba;

        gm.fromList(good_matches);

        List<KeyPoint> keypoints_objectList = keyPoint_train.toList();
        List<KeyPoint> keypoints_sceneList = keyPoint_test.toList();

        LinkedList<Point> objList = new LinkedList<Point>();
        LinkedList<Point> sceneList = new LinkedList<Point>();

        //將匹配到的特徵點取出
        for(int i = 0; i<good_matches.size(); i++){
            Log.i("point1",""+keypoints_objectList.get(good_matches.get(i).queryIdx).pt);
            Log.i("point2",""+keypoints_sceneList.get(good_matches.get(i).trainIdx).pt);
            objList.addLast(keypoints_objectList.get(good_matches.get(i).queryIdx).pt);
            sceneList.addLast(keypoints_sceneList.get(good_matches.get(i).trainIdx).pt);
            Imgproc.circle(mRgba,keypoints_sceneList.get(good_matches.get(i).trainIdx).pt,2,new Scalar(255, 0, 255), -1);
        }

        MatOfPoint2f obj = new MatOfPoint2f();
        obj.fromList(objList);

        MatOfPoint2f scene = new MatOfPoint2f();
        scene.fromList(sceneList);

        //找出實景跟maker間的homography
        Mat hg = Calib3d.findHomography(obj, scene,0,Calib3d.RANSAC);

        Mat obj_corners = new Mat(4,1,CvType.CV_32FC2);
        Mat scene_corners = new Mat(4,1,CvType.CV_32FC2);

        obj_corners.put(0, 0, new double[] {0,0});
        obj_corners.put(1, 0, new double[] {makerGray.cols(),0});
        obj_corners.put(2, 0, new double[] {makerGray.cols(),makerGray.rows()});
        obj_corners.put(3, 0, new double[] {0,makerGray.rows()});

        //利用homography和maker已知的4個角來推出maker在實景中的位置
        Core.perspectiveTransform(obj_corners,scene_corners, hg);

        //劃出實景中maker邊線
        Imgproc.line(mRgba, new Point(scene_corners.get(0,0)), new Point(scene_corners.get(1,0)), new Scalar(0, 255, 0),4);
        Imgproc.line(mRgba, new Point(scene_corners.get(1,0)), new Point(scene_corners.get(2,0)), new Scalar(0, 255, 0),4);
        Imgproc.line(mRgba, new Point(scene_corners.get(2,0)), new Point(scene_corners.get(3,0)), new Scalar(0, 255, 0),4);
        Imgproc.line(mRgba, new Point(scene_corners.get(3,0)), new Point(scene_corners.get(0,0)), new Scalar(0, 255, 0),4);

        //推出相機在世界座標系的位置
        List<Point3> makerList = new ArrayList<Point3>();
        for(int i = 0; i<good_matches.size(); i++){
            Log.i("XandY"," "+keypoints_objectList.get(good_matches.get(i).queryIdx).pt.x+" "+keypoints_objectList.get(good_matches.get(i).queryIdx).pt.y);
            makerList.add(new Point3( -(( keypoints_objectList.get(good_matches.get(i).queryIdx).pt.x-maker.cols()/2 )*200/maker.cols()),
                   0,(( keypoints_objectList.get(good_matches.get(i).queryIdx).pt.y-maker.rows()/2 )*200/maker.rows())));
        }
        MatOfPoint3f makerPoints =new MatOfPoint3f();
        makerPoints.fromList(makerList);
        Calib3d.solvePnPRansac(makerPoints,scene,cameraMatrix,distCoeffs,Rvec,Tvec);//CV_EPNP n>3
        //Calib3d.solvePnP(makerPoints,scene,cameraMatrix,distCoeffs,Rvec,Tvec,true,Calib3d.CV_EPNP);//CV_EPNP n>3
        Log.i("Tvec",""+Tvec.get(0,0)[0]+" "+Tvec.get(1,0)[0]+" "+Tvec.get(2,0)[0]);
        //Rvec有3個參數要傳給server,代表相機跟標記間的角度關西
        Log.i("Rvec",""+Rvec.get(0,0)[0]+" "+Rvec.get(1,0)[0]+" "+Rvec.get(2,0)[0]);
        Log.i("Angle",""+Rvec.get(0,0)[0]*180/Math.PI+" "+Rvec.get(1,0)[0]*180/Math.PI+" "+Rvec.get(2,0)[0]*180/Math.PI);
        //將rvec轉成矩陣
        Mat rotMat=new Mat(3,3,CvType.CV_32F);
        Calib3d.Rodrigues(Rvec,rotMat);
        //camera世界座標
        Mat result=new Mat();
        Core.gemm(rotMat.inv(),Tvec,-1,new Mat(),0,result);//result=alpha*src1*src2+beta*src3
        Log.i("cameraWorld", result.get(0,0)[0]+" "+result.get(1,0)[0]+" "+result.get(2,0)[0]);
        // right-handed coordinates system (OpenCV) to left-handed one (Unity)

        unityPosition= new Point3(result.get(0,0)[0],-result.get(1,0)[0],result.get(2,0)[0]);

        Log.i("unityPosition", unityPosition.x+" "+unityPosition.y+" "+unityPosition.z);

        btnSend.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // 利用线程池直接开启一个线程 & 执行该线程
                mThreadPool.execute(new Runnable() {
                    @Override
                    public void run() {
                        while(true) {
                            //st = x + " " + y + " " + z + " " + viewX + " " + viewY + " " + viewZ + " " + playerList + " " + moveX + " " + moveY + " ";
                            st = unityPosition.x/10 + " " + unityPosition.y/10 + " " + unityPosition.z/10 + " " + decimalFormat.format(Rvec.get(0,0)[0]) + " " + decimalFormat.format(Rvec.get(1,0)[0]) + " " + decimalFormat.format(Rvec.get(2,0)[0]) + " " + playerList + " " + moveX + " " + moveY + " ";
                            try {
                                //從socket獲得輸出流outputStream
                                outputStream = socket.getOutputStream();
                                //寫入數據到輸出流
                                outputStream.write((st).getBytes("utf-8"));
                                // 特别注意：数据的结尾加上换行符才可让服务器端的readline()停止阻塞
                                //發送
                                outputStream.flush();
                                //重置move
                                moveX = 0;
                                moveY = 0;

                                System.out.println("開始接收檔案");
                                DataInputStream dataInput = new DataInputStream(socket.getInputStream());
                                int datasize = dataInput.available();
                                System.out.println("大小: " + datasize);
                                //互換buffer----------------------------------------------------
                                if(datasize > 0) { buffer = java.lang.Math.abs(buffer-1); }
                                byte[][] data = new byte[2][];
                                data[buffer] = new byte[datasize];
                                System.out.println("buffer = " + buffer);
                                //--------------------------------------------------------------
                                int len = 0;
                                while (len < datasize) {
                                    len += dataInput.read(data[buffer], len, datasize - len);
                                }
                                try {
                                    Thread.sleep(160);
                                } catch (InterruptedException ex) {
                                    Thread.currentThread().interrupt();
                                }
                                System.out.println("接收大小 : " + len);
                                System.out.println("接收檔案完成");
                                if(len < 100) continue;
                                //bmp = BitmapFactory.decodeByteArray(data[buffer], 0, data[buffer].length); //need thread to complete this step
                                /*bmp32 = bmp.copy(bmp.getConfig(), true);
                                Utils.bitmapToMat(bmp32, oldpaste);*/
                                paste = Imgcodecs.imdecode(new MatOfByte(data[buffer]), Imgcodecs.CV_LOAD_IMAGE_UNCHANGED);

                                //paste.put(0,0, data[buffer]);
                                //Imgproc.resize(mat, paste,size);//將圖片大小設為跟maker一樣*/
                                /*Message msg = Message.obtain();
                                mMainHandler.sendMessage(msg);*/

                                System.out.println("放置圖片完成");

                            } catch (IOException e) {
                                e.printStackTrace();
                            }
                        }
                    }
                });

            }
        });
/*
        //將obj與實景貼合
        for(int height=0;height<paste.rows();height++){
            for(int width=0;width<paste.cols();width++){
                //將貼圖(height,width)位置存下來
                obj_pixel.put(height, width, new double[] {height,width});
            }
        }
        //將貼圖(height,width)位置轉換成在實景的位置,存在scene_pixel中
        Core.perspectiveTransform(obj_pixel,scene_pixel, hg);
        //將obj的每一個pixel貼到實景上
        for(int height=0;height<paste.rows();height++){
            for(int width=0;width<paste.cols();width++){
                //將貼圖(height,width)位置的pixel的資料存下來
                double[] data=paste.get(height,width);
                Point point=new Point(scene_pixel.get(height,width));
                if(point.x>mRgba.cols()||point.x<0||point.y>mRgba.rows()||point.y<0)
                    continue;
                Log.i("pixelPosition",""+point.x+point.y);
                //Log.i("color","R:"+data[0]+" G:"+data[1]+" B:"+data[2]+" A:"+data[3]);
                //point.y對應row，point.x對應col,data[0-3] RGBA
                if(data[0]==255 && data[1]==255 && data[2]==255)//將obj白色部份去掉
                    continue;
                mRgba.put((int)point.y,(int)point.x,data[0],data[1],data[2],data[3]);
            }
        }*/
        return mRgba;
    }
}