#pragma once
#define _USE_MATH_DEFINES
#include <Windows.h>
#include"math.h"
#include <fstream>
#include<vector>
#include"Pt.h"
#include "CTBox.h"
#include"opencv2\opencv.hpp"
#include"..\DeviceEnum\DeviceEnumerator.h"
#include<time.h>


namespace WinForm_LRSensor {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO;
	using namespace System::IO::Ports;
	using namespace System::Runtime::InteropServices;
	using namespace System::Media;
	using namespace cv;
	using namespace std;
	double LIDAR_X_cooridate[361] = { 0 };
	double LIDAR_Y_cooridate[361] = { 0 };
	double LIDAR_R_cooridate[361] = { 0 };
	uint ComPortNoRecord[3] = { 0 };
	uint counter = 0;
	fstream ConnectRecord;
	VideoCapture cap;
	DeviceEnumerator de;
	vector<Pt>Pt_oldClusterRefPoint;
	std::map<int, Device> devices = de.getVideoDevicesMap();
	uint format = 25;
	double bsdAngle = 0;
	Pt AngleRadar_Point;
	float AlphaBias;
	double targetDistant;
	int LiDAR_Data[722] = { 0 };
	CTBox TBox;
	double PartitionValue = 0;
	Pt left_Radar_bias;
	Pt right_Radar_bias;
	Radar RadarData;
	std::string FileNameTime;
	
	int f_model_changed = 0;
	VideoWriter videoWrite;
	RNG rng(12345);
	time_t t1;
	fstream fp_LiDarReader;
	/// <summary>
	/// MyForm 的摘要 b 
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO:  在此加入建構函式程式碼
			timer1->Interval = 10;
			timer1->Start();
			ComPortRefresh();
			chart1->Show();
			LoadData();
			LoadComPort();
			char timeNow[30] = { 0 };
			uint currnetTime = System::DateTime::Now.Minute * 10000 + System::DateTime::Now.Second * 100 + System::DateTime::Now.Millisecond;
			sprintf(timeNow, "%d", currnetTime);
			FileNameTime = (string)"RecordData" + (string)timeNow;
			std::string str = (string)"mkdir " + FileNameTime;
			system(str.c_str());
			str = FileNameTime + (string)"\\VideoTest.avi";
			PartitionValue = Convert::ToDouble(tBox_Partition->Text) * 100;
		
		}

	protected:
		/// <summary>
		/// 清除任何使用中的資源。
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::TabControl^  tabControl1;
	protected:
	private: System::Windows::Forms::TabPage^  tabPage1;
	private: System::Windows::Forms::PictureBox^  pictureBox1;
	private: System::Windows::Forms::TabPage^  tabPage8;
	private: System::IO::Ports::SerialPort^  serialPort_LiDAR;
	private: System::ComponentModel::IContainer^  components;
	private:
		/// <summary>
		/// 設計工具所需的變數。
		/// </summary>
		double CurrentSpeed = 0;
		uint LiDARbufferIndex = 0;
		bool f_getLiDARData = false;
		bool f_getRRadarBias;
		bool f_getHeader = false;
	private: System::Windows::Forms::Timer^  timer1;
	private: System::IO::Ports::SerialPort^  serialPort_Radar_Angle;
	private: System::IO::Ports::SerialPort^  serialPort_Radar;
	private: System::IO::Ports::SerialPort^  serialPort_Tbox;
	private: System::Windows::Forms::Label^  tx_TBox_LAngle;
	private: System::Windows::Forms::Label^  tx_TBox_RAngle;
	private: System::Windows::Forms::Label^  Tx_CarSpeed;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  chart1;
	private: System::Windows::Forms::TabPage^  tabPage2;
	private: System::Windows::Forms::Button^  btn_RecordCnt;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::Button^  Btn_CamCnt;
	private: System::Windows::Forms::ComboBox^  cBox_CameraList;
	private: System::Windows::Forms::GroupBox^  groupBox5;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::Button^  Btn_Radar_Connect;
	private: System::Windows::Forms::ComboBox^  cBox_Radar;
	private: System::Windows::Forms::Button^  Btn_Refresh_Combox;
	private: System::Windows::Forms::GroupBox^  groupBox8;
	private: System::Windows::Forms::Button^  Btn_UpDateSetting;
	private: System::Windows::Forms::GroupBox^  groupBox10;
	private: System::Windows::Forms::Label^  label23;
	private: System::Windows::Forms::Label^  label24;
	private: System::Windows::Forms::TextBox^  txBox_targetDistant;
	private: System::Windows::Forms::TextBox^  txBox_AlphaBias;
	private: System::Windows::Forms::Label^  label25;
	private: System::Windows::Forms::Button^  Btn_Send_RadarAngle_Cmd;
	private: System::Windows::Forms::Button^  button6;
	private: System::Windows::Forms::Label^  lbBsdAngleT;
	private: System::Windows::Forms::GroupBox^  groupBox7;
	private: System::Windows::Forms::TabControl^  tabControl3;
	private: System::Windows::Forms::TabPage^  tabPage5;
	private: System::Windows::Forms::Label^  tx_LRadarBias_Y;
	private: System::Windows::Forms::Label^  tx_LRadarBias_X;
	private: System::Windows::Forms::Label^  label13;
	private: System::Windows::Forms::Label^  label14;
	private: System::Windows::Forms::Button^  Btn_LeftBias;
	private: System::Windows::Forms::TabPage^  tabPage6;
	private: System::Windows::Forms::CheckBox^  ckBox_RadarR;
	private: System::Windows::Forms::Label^  tx_RRadarBias_X;
	private: System::Windows::Forms::Label^  tx_RRadarBias_Y;
	private: System::Windows::Forms::Label^  label17;
	private: System::Windows::Forms::Label^  label18;
	private: System::Windows::Forms::Button^  Btn_RightBias;
	private: System::Windows::Forms::Label^  label20;
	private: System::Windows::Forms::Button^  Btn_RadarAngle_Connect;
	private: System::Windows::Forms::ComboBox^  cBox_Radar_Angle;
	private: System::Windows::Forms::GroupBox^  groupBox9;
	private: System::Windows::Forms::Label^  label21;
	private: System::Windows::Forms::ComboBox^  cBox_LiDAR;
	private: System::Windows::Forms::Label^  label22;
	private: System::Windows::Forms::TextBox^  tBox_Partition;
	private: System::Windows::Forms::Button^  Btn_LiDARCnt;
	private: System::Windows::Forms::Button^  Btn_LiDARClose;
	private: System::Windows::Forms::GroupBox^  groupBox6;
	private: System::Windows::Forms::TabControl^  tabControl2;
	private: System::Windows::Forms::TabPage^  tabPage3;
	private: System::Windows::Forms::TabPage^  tabPage4;
	private: System::Windows::Forms::TabPage^  tabPage7;
	private: System::Windows::Forms::Button^  Btn_TboxClose;
	private: System::Windows::Forms::Label^  Tx_Radar_Mode;
	private: System::Windows::Forms::ComboBox^  cBox_TBox;
	private: System::Windows::Forms::Button^  Btn_TboxCnt;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::DataGridView^  Table_BSD;

	private: System::Windows::Forms::DataGridViewTextBoxColumn^  Column5;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  Column6;
	private: System::Windows::Forms::DataGridView^  dataGridView1;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  dataGridViewTextBoxColumn1;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  dataGridViewTextBoxColumn2;
	private: System::Windows::Forms::DataGridView^  dataGridView3;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  dataGridViewTextBoxColumn3;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  chart2;
	private: System::Windows::Forms::Button^  Btn_PlayPause;
	private: System::Windows::Forms::PictureBox^  pictureBox2;
	private: System::Windows::Forms::Timer^  timer2;
	private: System::Windows::Forms::Label^  Tx_CarSpeed2;
private: System::Windows::Forms::ComboBox^  cBox_LIDAR_Mode;

private: System::Windows::Forms::Label^  label1;
private: System::Windows::Forms::Button^  Btn_UpDateFileName;

	private: System::Windows::Forms::DataGridViewTextBoxColumn^  dataGridViewTextBoxColumn4;




#pragma region Windows Form Designer generated code
			 /// <summary>
			 /// 此為設計工具支援所需的方法 - 請勿使用程式碼編輯器修改
			 /// 這個方法的內容。
			 /// </summary>
			 void InitializeComponent(void)
			 {
				 this->components = (gcnew System::ComponentModel::Container());
				 System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea1 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
				 System::Windows::Forms::DataVisualization::Charting::Legend^  legend1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series2 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series3 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series4 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series5 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series6 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(MyForm::typeid));
				 System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea2 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
				 System::Windows::Forms::DataVisualization::Charting::Legend^  legend2 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series7 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series8 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series9 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series10 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series11 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series12 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 this->tabControl1 = (gcnew System::Windows::Forms::TabControl());
				 this->tabPage1 = (gcnew System::Windows::Forms::TabPage());
				 this->Btn_UpDateFileName = (gcnew System::Windows::Forms::Button());
				 this->label7 = (gcnew System::Windows::Forms::Label());
				 this->chart1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
				 this->Tx_CarSpeed = (gcnew System::Windows::Forms::Label());
				 this->tx_TBox_LAngle = (gcnew System::Windows::Forms::Label());
				 this->tx_TBox_RAngle = (gcnew System::Windows::Forms::Label());
				 this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
				 this->tabPage2 = (gcnew System::Windows::Forms::TabPage());
				 this->btn_RecordCnt = (gcnew System::Windows::Forms::Button());
				 this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
				 this->Btn_CamCnt = (gcnew System::Windows::Forms::Button());
				 this->cBox_CameraList = (gcnew System::Windows::Forms::ComboBox());
				 this->groupBox5 = (gcnew System::Windows::Forms::GroupBox());
				 this->label9 = (gcnew System::Windows::Forms::Label());
				 this->Btn_Radar_Connect = (gcnew System::Windows::Forms::Button());
				 this->cBox_Radar = (gcnew System::Windows::Forms::ComboBox());
				 this->Btn_Refresh_Combox = (gcnew System::Windows::Forms::Button());
				 this->groupBox8 = (gcnew System::Windows::Forms::GroupBox());
				 this->groupBox10 = (gcnew System::Windows::Forms::GroupBox());
				 this->label23 = (gcnew System::Windows::Forms::Label());
				 this->label24 = (gcnew System::Windows::Forms::Label());
				 this->txBox_targetDistant = (gcnew System::Windows::Forms::TextBox());
				 this->txBox_AlphaBias = (gcnew System::Windows::Forms::TextBox());
				 this->label25 = (gcnew System::Windows::Forms::Label());
				 this->Btn_Send_RadarAngle_Cmd = (gcnew System::Windows::Forms::Button());
				 this->button6 = (gcnew System::Windows::Forms::Button());
				 this->lbBsdAngleT = (gcnew System::Windows::Forms::Label());
				 this->groupBox7 = (gcnew System::Windows::Forms::GroupBox());
				 this->tabControl3 = (gcnew System::Windows::Forms::TabControl());
				 this->tabPage5 = (gcnew System::Windows::Forms::TabPage());
				 this->tx_LRadarBias_Y = (gcnew System::Windows::Forms::Label());
				 this->tx_LRadarBias_X = (gcnew System::Windows::Forms::Label());
				 this->label13 = (gcnew System::Windows::Forms::Label());
				 this->label14 = (gcnew System::Windows::Forms::Label());
				 this->Btn_LeftBias = (gcnew System::Windows::Forms::Button());
				 this->tabPage6 = (gcnew System::Windows::Forms::TabPage());
				 this->ckBox_RadarR = (gcnew System::Windows::Forms::CheckBox());
				 this->tx_RRadarBias_X = (gcnew System::Windows::Forms::Label());
				 this->tx_RRadarBias_Y = (gcnew System::Windows::Forms::Label());
				 this->label17 = (gcnew System::Windows::Forms::Label());
				 this->label18 = (gcnew System::Windows::Forms::Label());
				 this->Btn_RightBias = (gcnew System::Windows::Forms::Button());
				 this->label20 = (gcnew System::Windows::Forms::Label());
				 this->Btn_RadarAngle_Connect = (gcnew System::Windows::Forms::Button());
				 this->cBox_Radar_Angle = (gcnew System::Windows::Forms::ComboBox());
				 this->groupBox9 = (gcnew System::Windows::Forms::GroupBox());
				 this->Btn_UpDateSetting = (gcnew System::Windows::Forms::Button());
				 this->cBox_LIDAR_Mode = (gcnew System::Windows::Forms::ComboBox());
				 this->label1 = (gcnew System::Windows::Forms::Label());
				 this->label21 = (gcnew System::Windows::Forms::Label());
				 this->cBox_LiDAR = (gcnew System::Windows::Forms::ComboBox());
				 this->label22 = (gcnew System::Windows::Forms::Label());
				 this->tBox_Partition = (gcnew System::Windows::Forms::TextBox());
				 this->Btn_LiDARCnt = (gcnew System::Windows::Forms::Button());
				 this->Btn_LiDARClose = (gcnew System::Windows::Forms::Button());
				 this->groupBox6 = (gcnew System::Windows::Forms::GroupBox());
				 this->tabControl2 = (gcnew System::Windows::Forms::TabControl());
				 this->tabPage3 = (gcnew System::Windows::Forms::TabPage());
				 this->Table_BSD = (gcnew System::Windows::Forms::DataGridView());
				 this->Column5 = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
				 this->Column6 = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
				 this->tabPage4 = (gcnew System::Windows::Forms::TabPage());
				 this->dataGridView1 = (gcnew System::Windows::Forms::DataGridView());
				 this->dataGridViewTextBoxColumn1 = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
				 this->dataGridViewTextBoxColumn2 = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
				 this->tabPage7 = (gcnew System::Windows::Forms::TabPage());
				 this->dataGridView3 = (gcnew System::Windows::Forms::DataGridView());
				 this->dataGridViewTextBoxColumn3 = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
				 this->dataGridViewTextBoxColumn4 = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
				 this->Btn_TboxClose = (gcnew System::Windows::Forms::Button());
				 this->Tx_Radar_Mode = (gcnew System::Windows::Forms::Label());
				 this->cBox_TBox = (gcnew System::Windows::Forms::ComboBox());
				 this->Btn_TboxCnt = (gcnew System::Windows::Forms::Button());
				 this->label3 = (gcnew System::Windows::Forms::Label());
				 this->label6 = (gcnew System::Windows::Forms::Label());
				 this->tabPage8 = (gcnew System::Windows::Forms::TabPage());
				 this->Tx_CarSpeed2 = (gcnew System::Windows::Forms::Label());
				 this->Btn_PlayPause = (gcnew System::Windows::Forms::Button());
				 this->pictureBox2 = (gcnew System::Windows::Forms::PictureBox());
				 this->chart2 = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
				 this->serialPort_LiDAR = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
				 this->serialPort_Radar_Angle = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->serialPort_Radar = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->serialPort_Tbox = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->timer2 = (gcnew System::Windows::Forms::Timer(this->components));
				 this->tabControl1->SuspendLayout();
				 this->tabPage1->SuspendLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart1))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
				 this->tabPage2->SuspendLayout();
				 this->groupBox1->SuspendLayout();
				 this->groupBox5->SuspendLayout();
				 this->groupBox8->SuspendLayout();
				 this->groupBox10->SuspendLayout();
				 this->groupBox7->SuspendLayout();
				 this->tabControl3->SuspendLayout();
				 this->tabPage5->SuspendLayout();
				 this->tabPage6->SuspendLayout();
				 this->groupBox9->SuspendLayout();
				 this->groupBox6->SuspendLayout();
				 this->tabControl2->SuspendLayout();
				 this->tabPage3->SuspendLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->Table_BSD))->BeginInit();
				 this->tabPage4->SuspendLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->dataGridView1))->BeginInit();
				 this->tabPage7->SuspendLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->dataGridView3))->BeginInit();
				 this->tabPage8->SuspendLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart2))->BeginInit();
				 this->SuspendLayout();
				 // 
				 // tabControl1
				 // 
				 this->tabControl1->Controls->Add(this->tabPage1);
				 this->tabControl1->Controls->Add(this->tabPage2);
				 this->tabControl1->Controls->Add(this->tabPage8);
				 this->tabControl1->Font = (gcnew System::Drawing::Font(L"新細明體", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(136)));
				 this->tabControl1->Location = System::Drawing::Point(12, -1);
				 this->tabControl1->Name = L"tabControl1";
				 this->tabControl1->SelectedIndex = 0;
				 this->tabControl1->Size = System::Drawing::Size(1890, 950);
				 this->tabControl1->TabIndex = 0;
				 // 
				 // tabPage1
				 // 
				 this->tabPage1->BackColor = System::Drawing::Color::Transparent;
				 this->tabPage1->Controls->Add(this->Btn_UpDateFileName);
				 this->tabPage1->Controls->Add(this->label7);
				 this->tabPage1->Controls->Add(this->chart1);
				 this->tabPage1->Controls->Add(this->Tx_CarSpeed);
				 this->tabPage1->Controls->Add(this->tx_TBox_LAngle);
				 this->tabPage1->Controls->Add(this->tx_TBox_RAngle);
				 this->tabPage1->Controls->Add(this->pictureBox1);
				 this->tabPage1->Location = System::Drawing::Point(4, 22);
				 this->tabPage1->Name = L"tabPage1";
				 this->tabPage1->Padding = System::Windows::Forms::Padding(3);
				 this->tabPage1->Size = System::Drawing::Size(1882, 924);
				 this->tabPage1->TabIndex = 0;
				 this->tabPage1->Text = L"數據";
				 // 
				 // Btn_UpDateFileName
				 // 
				 this->Btn_UpDateFileName->Location = System::Drawing::Point(1410, 526);
				 this->Btn_UpDateFileName->Name = L"Btn_UpDateFileName";
				 this->Btn_UpDateFileName->Size = System::Drawing::Size(75, 23);
				 this->Btn_UpDateFileName->TabIndex = 12;
				 this->Btn_UpDateFileName->Text = L"button1";
				 this->Btn_UpDateFileName->UseVisualStyleBackColor = true;
				 this->Btn_UpDateFileName->Click += gcnew System::EventHandler(this, &MyForm::Btn_UpDateFileName_Click);
				 // 
				 // label7
				 // 
				 this->label7->AutoSize = true;
				 this->label7->Location = System::Drawing::Point(1423, 661);
				 this->label7->Name = L"label7";
				 this->label7->Size = System::Drawing::Size(53, 12);
				 this->label7->TabIndex = 11;
				 this->label7->Text = L"CarSpeed:";
				 // 
				 // chart1
				 // 
				 chartArea1->AxisX->Interval = 100;
				 chartArea1->AxisX->Maximum = 1000;
				 chartArea1->AxisX->Minimum = -1000;
				 chartArea1->AxisY->Interval = 100;
				 chartArea1->AxisY->Maximum = 6000;
				 chartArea1->AxisY->Minimum = 0;
				 chartArea1->Name = L"ChartArea1";
				 this->chart1->ChartAreas->Add(chartArea1);
				 legend1->Name = L"Legend1";
				 this->chart1->Legends->Add(legend1);
				 this->chart1->Location = System::Drawing::Point(-42, -10);
				 this->chart1->Name = L"chart1";
				 series1->ChartArea = L"ChartArea1";
				 series1->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series1->Color = System::Drawing::Color::Khaki;
				 series1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 18, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 series1->LabelForeColor = System::Drawing::Color::YellowGreen;
				 series1->Legend = L"Legend1";
				 series1->Name = L"Series_LiDAR";
				 series2->ChartArea = L"ChartArea1";
				 series2->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series2->Color = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)),
					 static_cast<System::Int32>(static_cast<System::Byte>(192)));
				 series2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 series2->Legend = L"Legend1";
				 series2->MarkerColor = System::Drawing::Color::Blue;
				 series2->MarkerSize = 10;
				 series2->Name = L"Series_LiDAR_CLOSE";
				 series3->ChartArea = L"ChartArea1";
				 series3->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::FastPoint;
				 series3->Color = System::Drawing::Color::ForestGreen;
				 series3->Legend = L"Legend1";
				 series3->MarkerSize = 10;
				 series3->Name = L"Series_Radar_Angle";
				 series4->ChartArea = L"ChartArea1";
				 series4->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series4->Legend = L"Legend1";
				 series4->MarkerColor = System::Drawing::SystemColors::MenuHighlight;
				 series4->MarkerSize = 10;
				 series4->MarkerStyle = System::Windows::Forms::DataVisualization::Charting::MarkerStyle::Star4;
				 series4->Name = L"Series_TBox_LRadar";
				 series5->ChartArea = L"ChartArea1";
				 series5->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series5->Color = System::Drawing::Color::Black;
				 series5->Legend = L"Legend1";
				 series5->MarkerSize = 10;
				 series5->Name = L"Series_TBox_RRadar";
				 series6->ChartArea = L"ChartArea1";
				 series6->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series6->Legend = L"Legend1";
				 series6->MarkerSize = 10;
				 series6->Name = L"Series_RadarDetectArea";
				 this->chart1->Series->Add(series1);
				 this->chart1->Series->Add(series2);
				 this->chart1->Series->Add(series3);
				 this->chart1->Series->Add(series4);
				 this->chart1->Series->Add(series5);
				 this->chart1->Series->Add(series6);
				 this->chart1->Size = System::Drawing::Size(1389, 977);
				 this->chart1->TabIndex = 10;
				 this->chart1->Text = L"圖";
				 // 
				 // Tx_CarSpeed
				 // 
				 this->Tx_CarSpeed->AutoSize = true;
				 this->Tx_CarSpeed->Location = System::Drawing::Point(1482, 661);
				 this->Tx_CarSpeed->Name = L"Tx_CarSpeed";
				 this->Tx_CarSpeed->Size = System::Drawing::Size(33, 12);
				 this->Tx_CarSpeed->TabIndex = 3;
				 this->Tx_CarSpeed->Text = L"label7";
				 // 
				 // tx_TBox_LAngle
				 // 
				 this->tx_TBox_LAngle->AutoSize = true;
				 this->tx_TBox_LAngle->Location = System::Drawing::Point(1641, 591);
				 this->tx_TBox_LAngle->Name = L"tx_TBox_LAngle";
				 this->tx_TBox_LAngle->Size = System::Drawing::Size(33, 12);
				 this->tx_TBox_LAngle->TabIndex = 3;
				 this->tx_TBox_LAngle->Text = L"label7";
				 // 
				 // tx_TBox_RAngle
				 // 
				 this->tx_TBox_RAngle->AutoSize = true;
				 this->tx_TBox_RAngle->Location = System::Drawing::Point(1423, 591);
				 this->tx_TBox_RAngle->Name = L"tx_TBox_RAngle";
				 this->tx_TBox_RAngle->Size = System::Drawing::Size(33, 12);
				 this->tx_TBox_RAngle->TabIndex = 2;
				 this->tx_TBox_RAngle->Text = L"label7";
				 // 
				 // pictureBox1
				 // 
				 this->pictureBox1->Location = System::Drawing::Point(1353, 0);
				 this->pictureBox1->Name = L"pictureBox1";
				 this->pictureBox1->Size = System::Drawing::Size(487, 384);
				 this->pictureBox1->TabIndex = 1;
				 this->pictureBox1->TabStop = false;
				 // 
				 // tabPage2
				 // 
				 this->tabPage2->BackColor = System::Drawing::Color::Transparent;
				 this->tabPage2->Controls->Add(this->btn_RecordCnt);
				 this->tabPage2->Controls->Add(this->groupBox1);
				 this->tabPage2->Controls->Add(this->groupBox5);
				 this->tabPage2->Controls->Add(this->Btn_Refresh_Combox);
				 this->tabPage2->Controls->Add(this->groupBox8);
				 this->tabPage2->Controls->Add(this->groupBox9);
				 this->tabPage2->Controls->Add(this->groupBox6);
				 this->tabPage2->Font = (gcnew System::Drawing::Font(L"細明體", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(136)));
				 this->tabPage2->Location = System::Drawing::Point(4, 22);
				 this->tabPage2->Name = L"tabPage2";
				 this->tabPage2->Padding = System::Windows::Forms::Padding(3);
				 this->tabPage2->Size = System::Drawing::Size(1882, 924);
				 this->tabPage2->TabIndex = 1;
				 this->tabPage2->Text = L"設定";
				 // 
				 // btn_RecordCnt
				 // 
				 this->btn_RecordCnt->Location = System::Drawing::Point(249, 577);
				 this->btn_RecordCnt->Name = L"btn_RecordCnt";
				 this->btn_RecordCnt->Size = System::Drawing::Size(221, 106);
				 this->btn_RecordCnt->TabIndex = 15;
				 this->btn_RecordCnt->Text = L"Record Connect";
				 this->btn_RecordCnt->UseVisualStyleBackColor = true;
				 this->btn_RecordCnt->Click += gcnew System::EventHandler(this, &MyForm::btn_RecordCnt_Click);
				 // 
				 // groupBox1
				 // 
				 this->groupBox1->Controls->Add(this->Btn_CamCnt);
				 this->groupBox1->Controls->Add(this->cBox_CameraList);
				 this->groupBox1->Location = System::Drawing::Point(742, 120);
				 this->groupBox1->Name = L"groupBox1";
				 this->groupBox1->Size = System::Drawing::Size(150, 139);
				 this->groupBox1->TabIndex = 14;
				 this->groupBox1->TabStop = false;
				 this->groupBox1->Text = L"Camera";
				 // 
				 // Btn_CamCnt
				 // 
				 this->Btn_CamCnt->Location = System::Drawing::Point(6, 59);
				 this->Btn_CamCnt->Name = L"Btn_CamCnt";
				 this->Btn_CamCnt->Size = System::Drawing::Size(118, 66);
				 this->Btn_CamCnt->TabIndex = 13;
				 this->Btn_CamCnt->Text = L"開啟攝影機";
				 this->Btn_CamCnt->UseVisualStyleBackColor = true;
				 this->Btn_CamCnt->Click += gcnew System::EventHandler(this, &MyForm::Btn_CamCnt_Click);
				 // 
				 // cBox_CameraList
				 // 
				 this->cBox_CameraList->FormattingEnabled = true;
				 this->cBox_CameraList->Location = System::Drawing::Point(6, 21);
				 this->cBox_CameraList->Name = L"cBox_CameraList";
				 this->cBox_CameraList->Size = System::Drawing::Size(138, 20);
				 this->cBox_CameraList->TabIndex = 12;
				 // 
				 // groupBox5
				 // 
				 this->groupBox5->Controls->Add(this->label9);
				 this->groupBox5->Controls->Add(this->Btn_Radar_Connect);
				 this->groupBox5->Controls->Add(this->cBox_Radar);
				 this->groupBox5->Location = System::Drawing::Point(742, 18);
				 this->groupBox5->Name = L"groupBox5";
				 this->groupBox5->Size = System::Drawing::Size(200, 84);
				 this->groupBox5->TabIndex = 13;
				 this->groupBox5->TabStop = false;
				 this->groupBox5->Text = L"Radar";
				 // 
				 // label9
				 // 
				 this->label9->AutoSize = true;
				 this->label9->Location = System::Drawing::Point(18, 66);
				 this->label9->Name = L"label9";
				 this->label9->Size = System::Drawing::Size(41, 12);
				 this->label9->TabIndex = 14;
				 this->label9->Text = L"label9";
				 // 
				 // Btn_Radar_Connect
				 // 
				 this->Btn_Radar_Connect->Location = System::Drawing::Point(97, 19);
				 this->Btn_Radar_Connect->Name = L"Btn_Radar_Connect";
				 this->Btn_Radar_Connect->Size = System::Drawing::Size(75, 23);
				 this->Btn_Radar_Connect->TabIndex = 13;
				 this->Btn_Radar_Connect->Text = L"連接";
				 this->Btn_Radar_Connect->UseVisualStyleBackColor = true;
				 this->Btn_Radar_Connect->Click += gcnew System::EventHandler(this, &MyForm::Btn_Radar_Connect_Click);
				 // 
				 // cBox_Radar
				 // 
				 this->cBox_Radar->FormattingEnabled = true;
				 this->cBox_Radar->Location = System::Drawing::Point(6, 21);
				 this->cBox_Radar->Name = L"cBox_Radar";
				 this->cBox_Radar->Size = System::Drawing::Size(85, 20);
				 this->cBox_Radar->TabIndex = 12;
				 // 
				 // Btn_Refresh_Combox
				 // 
				 this->Btn_Refresh_Combox->Location = System::Drawing::Point(6, 577);
				 this->Btn_Refresh_Combox->Name = L"Btn_Refresh_Combox";
				 this->Btn_Refresh_Combox->Size = System::Drawing::Size(213, 106);
				 this->Btn_Refresh_Combox->TabIndex = 12;
				 this->Btn_Refresh_Combox->Text = L"Update Com";
				 this->Btn_Refresh_Combox->UseVisualStyleBackColor = true;
				 this->Btn_Refresh_Combox->Click += gcnew System::EventHandler(this, &MyForm::Btn_Refresh_Combox_Click);
				 // 
				 // groupBox8
				 // 
				 this->groupBox8->Controls->Add(this->groupBox10);
				 this->groupBox8->Controls->Add(this->Btn_Send_RadarAngle_Cmd);
				 this->groupBox8->Controls->Add(this->button6);
				 this->groupBox8->Controls->Add(this->lbBsdAngleT);
				 this->groupBox8->Controls->Add(this->groupBox7);
				 this->groupBox8->Controls->Add(this->label20);
				 this->groupBox8->Controls->Add(this->Btn_RadarAngle_Connect);
				 this->groupBox8->Controls->Add(this->cBox_Radar_Angle);
				 this->groupBox8->Location = System::Drawing::Point(342, 18);
				 this->groupBox8->Name = L"groupBox8";
				 this->groupBox8->Size = System::Drawing::Size(384, 292);
				 this->groupBox8->TabIndex = 11;
				 this->groupBox8->TabStop = false;
				 this->groupBox8->Text = L"Radar_Angle";
				 // 
				 // groupBox10
				 // 
				 this->groupBox10->Controls->Add(this->label23);
				 this->groupBox10->Controls->Add(this->label24);
				 this->groupBox10->Controls->Add(this->txBox_targetDistant);
				 this->groupBox10->Controls->Add(this->txBox_AlphaBias);
				 this->groupBox10->Controls->Add(this->label25);
				 this->groupBox10->Location = System::Drawing::Point(189, 20);
				 this->groupBox10->Margin = System::Windows::Forms::Padding(2);
				 this->groupBox10->Name = L"groupBox10";
				 this->groupBox10->Padding = System::Windows::Forms::Padding(2);
				 this->groupBox10->Size = System::Drawing::Size(181, 80);
				 this->groupBox10->TabIndex = 16;
				 this->groupBox10->TabStop = false;
				 this->groupBox10->Text = L"AngleRadarSetting";
				 // 
				 // label23
				 // 
				 this->label23->AutoSize = true;
				 this->label23->Location = System::Drawing::Point(158, 66);
				 this->label23->Name = L"label23";
				 this->label23->Size = System::Drawing::Size(23, 12);
				 this->label23->TabIndex = 14;
				 this->label23->Text = L"(m)";
				 // 
				 // label24
				 // 
				 this->label24->AutoSize = true;
				 this->label24->Location = System::Drawing::Point(5, 66);
				 this->label24->Name = L"label24";
				 this->label24->Size = System::Drawing::Size(95, 12);
				 this->label24->TabIndex = 13;
				 this->label24->Text = L"target Distant:";
				 // 
				 // txBox_targetDistant
				 // 
				 this->txBox_targetDistant->Location = System::Drawing::Point(106, 56);
				 this->txBox_targetDistant->Name = L"txBox_targetDistant";
				 this->txBox_targetDistant->Size = System::Drawing::Size(46, 22);
				 this->txBox_targetDistant->TabIndex = 12;
				 this->txBox_targetDistant->Text = L"3.24";
				 // 
				 // txBox_AlphaBias
				 // 
				 this->txBox_AlphaBias->Location = System::Drawing::Point(80, 16);
				 this->txBox_AlphaBias->Name = L"txBox_AlphaBias";
				 this->txBox_AlphaBias->Size = System::Drawing::Size(46, 22);
				 this->txBox_AlphaBias->TabIndex = 11;
				 this->txBox_AlphaBias->Text = L"-2.67";
				 // 
				 // label25
				 // 
				 this->label25->AutoSize = true;
				 this->label25->Location = System::Drawing::Point(5, 26);
				 this->label25->Name = L"label25";
				 this->label25->Size = System::Drawing::Size(71, 12);
				 this->label25->TabIndex = 10;
				 this->label25->Text = L"Alpha Bias:";
				 // 
				 // Btn_Send_RadarAngle_Cmd
				 // 
				 this->Btn_Send_RadarAngle_Cmd->Location = System::Drawing::Point(8, 62);
				 this->Btn_Send_RadarAngle_Cmd->Name = L"Btn_Send_RadarAngle_Cmd";
				 this->Btn_Send_RadarAngle_Cmd->Size = System::Drawing::Size(75, 23);
				 this->Btn_Send_RadarAngle_Cmd->TabIndex = 10;
				 this->Btn_Send_RadarAngle_Cmd->Text = L"Send BSD";
				 this->Btn_Send_RadarAngle_Cmd->UseVisualStyleBackColor = true;
				 this->Btn_Send_RadarAngle_Cmd->Click += gcnew System::EventHandler(this, &MyForm::Btn_Send_RadarAngle_Cmd_Click_1);
				 // 
				 // button6
				 // 
				 this->button6->Location = System::Drawing::Point(99, 62);
				 this->button6->Name = L"button6";
				 this->button6->Size = System::Drawing::Size(75, 23);
				 this->button6->TabIndex = 8;
				 this->button6->Text = L"關閉";
				 this->button6->UseVisualStyleBackColor = true;
				 // 
				 // lbBsdAngleT
				 // 
				 this->lbBsdAngleT->AutoSize = true;
				 this->lbBsdAngleT->Location = System::Drawing::Point(73, 119);
				 this->lbBsdAngleT->Name = L"lbBsdAngleT";
				 this->lbBsdAngleT->Size = System::Drawing::Size(41, 12);
				 this->lbBsdAngleT->TabIndex = 5;
				 this->lbBsdAngleT->Text = L"label3";
				 // 
				 // groupBox7
				 // 
				 this->groupBox7->Controls->Add(this->tabControl3);
				 this->groupBox7->Location = System::Drawing::Point(8, 156);
				 this->groupBox7->Name = L"groupBox7";
				 this->groupBox7->Size = System::Drawing::Size(195, 130);
				 this->groupBox7->TabIndex = 8;
				 this->groupBox7->TabStop = false;
				 this->groupBox7->Text = L"對位";
				 // 
				 // tabControl3
				 // 
				 this->tabControl3->Controls->Add(this->tabPage5);
				 this->tabControl3->Controls->Add(this->tabPage6);
				 this->tabControl3->Location = System::Drawing::Point(6, 21);
				 this->tabControl3->Name = L"tabControl3";
				 this->tabControl3->SelectedIndex = 0;
				 this->tabControl3->Size = System::Drawing::Size(166, 101);
				 this->tabControl3->TabIndex = 0;
				 // 
				 // tabPage5
				 // 
				 this->tabPage5->Controls->Add(this->tx_LRadarBias_Y);
				 this->tabPage5->Controls->Add(this->tx_LRadarBias_X);
				 this->tabPage5->Controls->Add(this->label13);
				 this->tabPage5->Controls->Add(this->label14);
				 this->tabPage5->Controls->Add(this->Btn_LeftBias);
				 this->tabPage5->Location = System::Drawing::Point(4, 22);
				 this->tabPage5->Name = L"tabPage5";
				 this->tabPage5->Padding = System::Windows::Forms::Padding(3);
				 this->tabPage5->Size = System::Drawing::Size(158, 75);
				 this->tabPage5->TabIndex = 0;
				 this->tabPage5->Text = L"左邊雷達";
				 this->tabPage5->UseVisualStyleBackColor = true;
				 // 
				 // tx_LRadarBias_Y
				 // 
				 this->tx_LRadarBias_Y->AutoSize = true;
				 this->tx_LRadarBias_Y->Location = System::Drawing::Point(92, 41);
				 this->tx_LRadarBias_Y->Name = L"tx_LRadarBias_Y";
				 this->tx_LRadarBias_Y->Size = System::Drawing::Size(41, 12);
				 this->tx_LRadarBias_Y->TabIndex = 3;
				 this->tx_LRadarBias_Y->Text = L"label7";
				 // 
				 // tx_LRadarBias_X
				 // 
				 this->tx_LRadarBias_X->AutoSize = true;
				 this->tx_LRadarBias_X->Location = System::Drawing::Point(92, 15);
				 this->tx_LRadarBias_X->Name = L"tx_LRadarBias_X";
				 this->tx_LRadarBias_X->Size = System::Drawing::Size(41, 12);
				 this->tx_LRadarBias_X->TabIndex = 2;
				 this->tx_LRadarBias_X->Text = L"label6";
				 // 
				 // label13
				 // 
				 this->label13->AutoSize = true;
				 this->label13->Location = System::Drawing::Point(69, 42);
				 this->label13->Name = L"label13";
				 this->label13->Size = System::Drawing::Size(17, 12);
				 this->label13->TabIndex = 1;
				 this->label13->Text = L"Y:";
				 // 
				 // label14
				 // 
				 this->label14->AutoSize = true;
				 this->label14->Location = System::Drawing::Point(69, 16);
				 this->label14->Name = L"label14";
				 this->label14->Size = System::Drawing::Size(17, 12);
				 this->label14->TabIndex = 1;
				 this->label14->Text = L"X:";
				 // 
				 // Btn_LeftBias
				 // 
				 this->Btn_LeftBias->Location = System::Drawing::Point(3, 6);
				 this->Btn_LeftBias->Name = L"Btn_LeftBias";
				 this->Btn_LeftBias->Size = System::Drawing::Size(58, 62);
				 this->Btn_LeftBias->TabIndex = 0;
				 this->Btn_LeftBias->Text = L"確定";
				 this->Btn_LeftBias->UseVisualStyleBackColor = true;
				 this->Btn_LeftBias->Click += gcnew System::EventHandler(this, &MyForm::Btn_LeftBias_Click);
				 // 
				 // tabPage6
				 // 
				 this->tabPage6->Controls->Add(this->ckBox_RadarR);
				 this->tabPage6->Controls->Add(this->tx_RRadarBias_X);
				 this->tabPage6->Controls->Add(this->tx_RRadarBias_Y);
				 this->tabPage6->Controls->Add(this->label17);
				 this->tabPage6->Controls->Add(this->label18);
				 this->tabPage6->Controls->Add(this->Btn_RightBias);
				 this->tabPage6->Location = System::Drawing::Point(4, 22);
				 this->tabPage6->Name = L"tabPage6";
				 this->tabPage6->Padding = System::Windows::Forms::Padding(3);
				 this->tabPage6->Size = System::Drawing::Size(158, 75);
				 this->tabPage6->TabIndex = 1;
				 this->tabPage6->Text = L"右邊雷達";
				 this->tabPage6->UseVisualStyleBackColor = true;
				 // 
				 // ckBox_RadarR
				 // 
				 this->ckBox_RadarR->AutoSize = true;
				 this->ckBox_RadarR->Location = System::Drawing::Point(67, 55);
				 this->ckBox_RadarR->Name = L"ckBox_RadarR";
				 this->ckBox_RadarR->Size = System::Drawing::Size(84, 16);
				 this->ckBox_RadarR->TabIndex = 9;
				 this->ckBox_RadarR->Text = L"雷達在右邊";
				 this->ckBox_RadarR->UseVisualStyleBackColor = true;
				 // 
				 // tx_RRadarBias_X
				 // 
				 this->tx_RRadarBias_X->AutoSize = true;
				 this->tx_RRadarBias_X->Location = System::Drawing::Point(91, 16);
				 this->tx_RRadarBias_X->Name = L"tx_RRadarBias_X";
				 this->tx_RRadarBias_X->Size = System::Drawing::Size(41, 12);
				 this->tx_RRadarBias_X->TabIndex = 8;
				 this->tx_RRadarBias_X->Text = L"label8";
				 // 
				 // tx_RRadarBias_Y
				 // 
				 this->tx_RRadarBias_Y->AutoSize = true;
				 this->tx_RRadarBias_Y->Location = System::Drawing::Point(91, 41);
				 this->tx_RRadarBias_Y->Name = L"tx_RRadarBias_Y";
				 this->tx_RRadarBias_Y->Size = System::Drawing::Size(41, 12);
				 this->tx_RRadarBias_Y->TabIndex = 7;
				 this->tx_RRadarBias_Y->Text = L"label9";
				 // 
				 // label17
				 // 
				 this->label17->AutoSize = true;
				 this->label17->Location = System::Drawing::Point(69, 41);
				 this->label17->Name = L"label17";
				 this->label17->Size = System::Drawing::Size(17, 12);
				 this->label17->TabIndex = 5;
				 this->label17->Text = L"Y:";
				 // 
				 // label18
				 // 
				 this->label18->AutoSize = true;
				 this->label18->Location = System::Drawing::Point(69, 16);
				 this->label18->Name = L"label18";
				 this->label18->Size = System::Drawing::Size(17, 12);
				 this->label18->TabIndex = 6;
				 this->label18->Text = L"X:";
				 // 
				 // Btn_RightBias
				 // 
				 this->Btn_RightBias->Location = System::Drawing::Point(3, 6);
				 this->Btn_RightBias->Name = L"Btn_RightBias";
				 this->Btn_RightBias->Size = System::Drawing::Size(58, 62);
				 this->Btn_RightBias->TabIndex = 4;
				 this->Btn_RightBias->Text = L"確定";
				 this->Btn_RightBias->UseVisualStyleBackColor = true;
				 this->Btn_RightBias->Click += gcnew System::EventHandler(this, &MyForm::Btn_RightBias_Click);
				 // 
				 // label20
				 // 
				 this->label20->AutoSize = true;
				 this->label20->Location = System::Drawing::Point(15, 119);
				 this->label20->Name = L"label20";
				 this->label20->Size = System::Drawing::Size(41, 12);
				 this->label20->TabIndex = 4;
				 this->label20->Text = L"Angle:";
				 // 
				 // Btn_RadarAngle_Connect
				 // 
				 this->Btn_RadarAngle_Connect->Location = System::Drawing::Point(99, 21);
				 this->Btn_RadarAngle_Connect->Name = L"Btn_RadarAngle_Connect";
				 this->Btn_RadarAngle_Connect->Size = System::Drawing::Size(75, 23);
				 this->Btn_RadarAngle_Connect->TabIndex = 1;
				 this->Btn_RadarAngle_Connect->Text = L"連接";
				 this->Btn_RadarAngle_Connect->UseVisualStyleBackColor = true;
				 // 
				 // cBox_Radar_Angle
				 // 
				 this->cBox_Radar_Angle->FormattingEnabled = true;
				 this->cBox_Radar_Angle->Location = System::Drawing::Point(6, 21);
				 this->cBox_Radar_Angle->Name = L"cBox_Radar_Angle";
				 this->cBox_Radar_Angle->Size = System::Drawing::Size(87, 20);
				 this->cBox_Radar_Angle->TabIndex = 0;
				 // 
				 // groupBox9
				 // 
				 this->groupBox9->Controls->Add(this->Btn_UpDateSetting);
				 this->groupBox9->Controls->Add(this->cBox_LIDAR_Mode);
				 this->groupBox9->Controls->Add(this->label1);
				 this->groupBox9->Controls->Add(this->label21);
				 this->groupBox9->Controls->Add(this->cBox_LiDAR);
				 this->groupBox9->Controls->Add(this->label22);
				 this->groupBox9->Controls->Add(this->tBox_Partition);
				 this->groupBox9->Controls->Add(this->Btn_LiDARCnt);
				 this->groupBox9->Controls->Add(this->Btn_LiDARClose);
				 this->groupBox9->Location = System::Drawing::Point(6, 18);
				 this->groupBox9->Name = L"groupBox9";
				 this->groupBox9->Size = System::Drawing::Size(241, 161);
				 this->groupBox9->TabIndex = 10;
				 this->groupBox9->TabStop = false;
				 this->groupBox9->Text = L"LiDAR";
				 // 
				 // Btn_UpDateSetting
				 // 
				 this->Btn_UpDateSetting->Location = System::Drawing::Point(157, 120);
				 this->Btn_UpDateSetting->Name = L"Btn_UpDateSetting";
				 this->Btn_UpDateSetting->Size = System::Drawing::Size(75, 23);
				 this->Btn_UpDateSetting->TabIndex = 17;
				 this->Btn_UpDateSetting->Text = L"更新設定值";
				 this->Btn_UpDateSetting->UseVisualStyleBackColor = true;
				 this->Btn_UpDateSetting->Click += gcnew System::EventHandler(this, &MyForm::Btn_UpDateSetting_Click);
				 // 
				 // cBox_LIDAR_Mode
				 // 
				 this->cBox_LIDAR_Mode->FormattingEnabled = true;
				 this->cBox_LIDAR_Mode->Location = System::Drawing::Point(29, 57);
				 this->cBox_LIDAR_Mode->Name = L"cBox_LIDAR_Mode";
				 this->cBox_LIDAR_Mode->Size = System::Drawing::Size(121, 20);
				 this->cBox_LIDAR_Mode->TabIndex = 16;
				 this->cBox_LIDAR_Mode->SelectedIndexChanged += gcnew System::EventHandler(this, &MyForm::cBox_LIDAR_Mode_SelectedIndexChanged);
				 // 
				 // label1
				 // 
				 this->label1->AutoSize = true;
				 this->label1->Location = System::Drawing::Point(27, 22);
				 this->label1->Name = L"label1";
				 this->label1->Size = System::Drawing::Size(29, 12);
				 this->label1->TabIndex = 15;
				 this->label1->Text = L"Com:";
				 // 
				 // label21
				 // 
				 this->label21->AutoSize = true;
				 this->label21->Location = System::Drawing::Point(126, 123);
				 this->label21->Margin = System::Windows::Forms::Padding(2, 0, 2, 0);
				 this->label21->Name = L"label21";
				 this->label21->Size = System::Drawing::Size(23, 12);
				 this->label21->TabIndex = 14;
				 this->label21->Text = L"(m)";
				 // 
				 // cBox_LiDAR
				 // 
				 this->cBox_LiDAR->FormattingEnabled = true;
				 this->cBox_LiDAR->Location = System::Drawing::Point(62, 17);
				 this->cBox_LiDAR->Name = L"cBox_LiDAR";
				 this->cBox_LiDAR->Size = System::Drawing::Size(87, 20);
				 this->cBox_LiDAR->TabIndex = 0;
				 // 
				 // label22
				 // 
				 this->label22->AutoSize = true;
				 this->label22->Location = System::Drawing::Point(12, 123);
				 this->label22->Margin = System::Windows::Forms::Padding(2, 0, 2, 0);
				 this->label22->Name = L"label22";
				 this->label22->Size = System::Drawing::Size(59, 12);
				 this->label22->TabIndex = 13;
				 this->label22->Text = L"聚類閥值:";
				 // 
				 // tBox_Partition
				 // 
				 this->tBox_Partition->Location = System::Drawing::Point(77, 113);
				 this->tBox_Partition->Margin = System::Windows::Forms::Padding(2);
				 this->tBox_Partition->Name = L"tBox_Partition";
				 this->tBox_Partition->Size = System::Drawing::Size(36, 22);
				 this->tBox_Partition->TabIndex = 12;
				 this->tBox_Partition->Text = L"1";
				 // 
				 // Btn_LiDARCnt
				 // 
				 this->Btn_LiDARCnt->Location = System::Drawing::Point(157, 17);
				 this->Btn_LiDARCnt->Name = L"Btn_LiDARCnt";
				 this->Btn_LiDARCnt->Size = System::Drawing::Size(75, 23);
				 this->Btn_LiDARCnt->TabIndex = 5;
				 this->Btn_LiDARCnt->Text = L"連接";
				 this->Btn_LiDARCnt->UseVisualStyleBackColor = true;
				 this->Btn_LiDARCnt->Click += gcnew System::EventHandler(this, &MyForm::Btn_LiDARCnt_Click);
				 // 
				 // Btn_LiDARClose
				 // 
				 this->Btn_LiDARClose->Location = System::Drawing::Point(157, 57);
				 this->Btn_LiDARClose->Name = L"Btn_LiDARClose";
				 this->Btn_LiDARClose->Size = System::Drawing::Size(75, 23);
				 this->Btn_LiDARClose->TabIndex = 6;
				 this->Btn_LiDARClose->Text = L"關閉";
				 this->Btn_LiDARClose->UseVisualStyleBackColor = true;
				 this->Btn_LiDARClose->Click += gcnew System::EventHandler(this, &MyForm::Btn_LiDARClose_Click);
				 // 
				 // groupBox6
				 // 
				 this->groupBox6->Controls->Add(this->tabControl2);
				 this->groupBox6->Controls->Add(this->Btn_TboxClose);
				 this->groupBox6->Controls->Add(this->Tx_Radar_Mode);
				 this->groupBox6->Controls->Add(this->cBox_TBox);
				 this->groupBox6->Controls->Add(this->Btn_TboxCnt);
				 this->groupBox6->Controls->Add(this->label3);
				 this->groupBox6->Controls->Add(this->label6);
				 this->groupBox6->Location = System::Drawing::Point(12, 325);
				 this->groupBox6->Name = L"groupBox6";
				 this->groupBox6->Size = System::Drawing::Size(481, 246);
				 this->groupBox6->TabIndex = 9;
				 this->groupBox6->TabStop = false;
				 this->groupBox6->Text = L"TBox";
				 // 
				 // tabControl2
				 // 
				 this->tabControl2->Controls->Add(this->tabPage3);
				 this->tabControl2->Controls->Add(this->tabPage4);
				 this->tabControl2->Controls->Add(this->tabPage7);
				 this->tabControl2->Location = System::Drawing::Point(204, 26);
				 this->tabControl2->Name = L"tabControl2";
				 this->tabControl2->SelectedIndex = 0;
				 this->tabControl2->Size = System::Drawing::Size(254, 209);
				 this->tabControl2->TabIndex = 15;
				 // 
				 // tabPage3
				 // 
				 this->tabPage3->Controls->Add(this->Table_BSD);
				 this->tabPage3->Location = System::Drawing::Point(4, 22);
				 this->tabPage3->Name = L"tabPage3";
				 this->tabPage3->Padding = System::Windows::Forms::Padding(3);
				 this->tabPage3->Size = System::Drawing::Size(246, 183);
				 this->tabPage3->TabIndex = 0;
				 this->tabPage3->Text = L"BSD";
				 this->tabPage3->UseVisualStyleBackColor = true;
				 // 
				 // Table_BSD
				 // 
				 this->Table_BSD->ColumnHeadersHeightSizeMode = System::Windows::Forms::DataGridViewColumnHeadersHeightSizeMode::AutoSize;
				 this->Table_BSD->Columns->AddRange(gcnew cli::array< System::Windows::Forms::DataGridViewColumn^  >(2) { this->Column5, this->Column6 });
				 this->Table_BSD->Location = System::Drawing::Point(0, 0);
				 this->Table_BSD->Name = L"Table_BSD";
				 this->Table_BSD->RowTemplate->Height = 24;
				 this->Table_BSD->Size = System::Drawing::Size(243, 77);
				 this->Table_BSD->TabIndex = 7;
				 // 
				 // Column5
				 // 
				 this->Column5->HeaderText = L"距離閥值(L)";
				 this->Column5->Name = L"Column5";
				 // 
				 // Column6
				 // 
				 this->Column6->HeaderText = L"距離閥值(H)";
				 this->Column6->Name = L"Column6";
				 // 
				 // tabPage4
				 // 
				 this->tabPage4->Controls->Add(this->dataGridView1);
				 this->tabPage4->Location = System::Drawing::Point(4, 22);
				 this->tabPage4->Name = L"tabPage4";
				 this->tabPage4->Padding = System::Windows::Forms::Padding(3);
				 this->tabPage4->Size = System::Drawing::Size(246, 183);
				 this->tabPage4->TabIndex = 1;
				 this->tabPage4->Text = L"RCTA";
				 this->tabPage4->UseVisualStyleBackColor = true;
				 // 
				 // dataGridView1
				 // 
				 this->dataGridView1->ColumnHeadersHeightSizeMode = System::Windows::Forms::DataGridViewColumnHeadersHeightSizeMode::AutoSize;
				 this->dataGridView1->Columns->AddRange(gcnew cli::array< System::Windows::Forms::DataGridViewColumn^  >(2) {
					 this->dataGridViewTextBoxColumn1,
						 this->dataGridViewTextBoxColumn2
				 });
				 this->dataGridView1->Location = System::Drawing::Point(0, 0);
				 this->dataGridView1->Name = L"dataGridView1";
				 this->dataGridView1->RowTemplate->Height = 24;
				 this->dataGridView1->Size = System::Drawing::Size(243, 77);
				 this->dataGridView1->TabIndex = 8;
				 // 
				 // dataGridViewTextBoxColumn1
				 // 
				 this->dataGridViewTextBoxColumn1->HeaderText = L"距離閥值(L)";
				 this->dataGridViewTextBoxColumn1->Name = L"dataGridViewTextBoxColumn1";
				 // 
				 // dataGridViewTextBoxColumn2
				 // 
				 this->dataGridViewTextBoxColumn2->HeaderText = L"距離閥值(H)";
				 this->dataGridViewTextBoxColumn2->Name = L"dataGridViewTextBoxColumn2";
				 // 
				 // tabPage7
				 // 
				 this->tabPage7->Controls->Add(this->dataGridView3);
				 this->tabPage7->Location = System::Drawing::Point(4, 22);
				 this->tabPage7->Name = L"tabPage7";
				 this->tabPage7->Size = System::Drawing::Size(246, 183);
				 this->tabPage7->TabIndex = 2;
				 this->tabPage7->Text = L"DOW";
				 this->tabPage7->UseVisualStyleBackColor = true;
				 // 
				 // dataGridView3
				 // 
				 this->dataGridView3->ColumnHeadersHeightSizeMode = System::Windows::Forms::DataGridViewColumnHeadersHeightSizeMode::AutoSize;
				 this->dataGridView3->Columns->AddRange(gcnew cli::array< System::Windows::Forms::DataGridViewColumn^  >(2) {
					 this->dataGridViewTextBoxColumn3,
						 this->dataGridViewTextBoxColumn4
				 });
				 this->dataGridView3->Location = System::Drawing::Point(0, 2);
				 this->dataGridView3->Name = L"dataGridView3";
				 this->dataGridView3->RowTemplate->Height = 24;
				 this->dataGridView3->Size = System::Drawing::Size(243, 77);
				 this->dataGridView3->TabIndex = 8;
				 // 
				 // dataGridViewTextBoxColumn3
				 // 
				 this->dataGridViewTextBoxColumn3->HeaderText = L"距離閥值(L)";
				 this->dataGridViewTextBoxColumn3->Name = L"dataGridViewTextBoxColumn3";
				 // 
				 // dataGridViewTextBoxColumn4
				 // 
				 this->dataGridViewTextBoxColumn4->HeaderText = L"距離閥值(H)";
				 this->dataGridViewTextBoxColumn4->Name = L"dataGridViewTextBoxColumn4";
				 // 
				 // Btn_TboxClose
				 // 
				 this->Btn_TboxClose->Location = System::Drawing::Point(99, 54);
				 this->Btn_TboxClose->Margin = System::Windows::Forms::Padding(2);
				 this->Btn_TboxClose->Name = L"Btn_TboxClose";
				 this->Btn_TboxClose->Size = System::Drawing::Size(75, 23);
				 this->Btn_TboxClose->TabIndex = 12;
				 this->Btn_TboxClose->Text = L"關閉";
				 this->Btn_TboxClose->UseVisualStyleBackColor = true;
				 // 
				 // Tx_Radar_Mode
				 // 
				 this->Tx_Radar_Mode->AutoSize = true;
				 this->Tx_Radar_Mode->Location = System::Drawing::Point(6, 98);
				 this->Tx_Radar_Mode->Name = L"Tx_Radar_Mode";
				 this->Tx_Radar_Mode->Size = System::Drawing::Size(29, 12);
				 this->Tx_Radar_Mode->TabIndex = 11;
				 this->Tx_Radar_Mode->Text = L"Mode";
				 // 
				 // cBox_TBox
				 // 
				 this->cBox_TBox->FormattingEnabled = true;
				 this->cBox_TBox->Location = System::Drawing::Point(0, 26);
				 this->cBox_TBox->Name = L"cBox_TBox";
				 this->cBox_TBox->Size = System::Drawing::Size(87, 20);
				 this->cBox_TBox->TabIndex = 7;
				 // 
				 // Btn_TboxCnt
				 // 
				 this->Btn_TboxCnt->Location = System::Drawing::Point(99, 21);
				 this->Btn_TboxCnt->Name = L"Btn_TboxCnt";
				 this->Btn_TboxCnt->Size = System::Drawing::Size(75, 23);
				 this->Btn_TboxCnt->TabIndex = 8;
				 this->Btn_TboxCnt->Text = L"連接";
				 this->Btn_TboxCnt->UseVisualStyleBackColor = true;
				 this->Btn_TboxCnt->Click += gcnew System::EventHandler(this, &MyForm::Btn_TboxCnt_Click);
				 // 
				 // label3
				 // 
				 this->label3->AutoSize = true;
				 this->label3->Location = System::Drawing::Point(6, 59);
				 this->label3->Name = L"label3";
				 this->label3->Size = System::Drawing::Size(59, 12);
				 this->label3->TabIndex = 9;
				 this->label3->Text = L"目前車速:";
				 // 
				 // label6
				 // 
				 this->label6->AutoSize = true;
				 this->label6->Location = System::Drawing::Point(82, 59);
				 this->label6->Name = L"label6";
				 this->label6->Size = System::Drawing::Size(11, 12);
				 this->label6->TabIndex = 10;
				 this->label6->Text = L"0";
				 // 
				 // tabPage8
				 // 
				 this->tabPage8->Controls->Add(this->Tx_CarSpeed2);
				 this->tabPage8->Controls->Add(this->Btn_PlayPause);
				 this->tabPage8->Controls->Add(this->pictureBox2);
				 this->tabPage8->Controls->Add(this->chart2);
				 this->tabPage8->Location = System::Drawing::Point(4, 22);
				 this->tabPage8->Name = L"tabPage8";
				 this->tabPage8->Size = System::Drawing::Size(1882, 924);
				 this->tabPage8->TabIndex = 2;
				 this->tabPage8->Text = L"回放";
				 this->tabPage8->UseVisualStyleBackColor = true;
				 // 
				 // Tx_CarSpeed2
				 // 
				 this->Tx_CarSpeed2->AutoSize = true;
				 this->Tx_CarSpeed2->Location = System::Drawing::Point(1346, 453);
				 this->Tx_CarSpeed2->Name = L"Tx_CarSpeed2";
				 this->Tx_CarSpeed2->Size = System::Drawing::Size(33, 12);
				 this->Tx_CarSpeed2->TabIndex = 14;
				 this->Tx_CarSpeed2->Text = L"label1";
				 // 
				 // Btn_PlayPause
				 // 
				 this->Btn_PlayPause->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"Btn_PlayPause.Image")));
				 this->Btn_PlayPause->Location = System::Drawing::Point(1537, 585);
				 this->Btn_PlayPause->Name = L"Btn_PlayPause";
				 this->Btn_PlayPause->Size = System::Drawing::Size(69, 69);
				 this->Btn_PlayPause->TabIndex = 13;
				 this->Btn_PlayPause->UseVisualStyleBackColor = true;
				 this->Btn_PlayPause->Click += gcnew System::EventHandler(this, &MyForm::Btn_PlayPause_Click);
				 // 
				 // pictureBox2
				 // 
				 this->pictureBox2->Location = System::Drawing::Point(1334, 3);
				 this->pictureBox2->Name = L"pictureBox2";
				 this->pictureBox2->Size = System::Drawing::Size(487, 384);
				 this->pictureBox2->TabIndex = 12;
				 this->pictureBox2->TabStop = false;
				 // 
				 // chart2
				 // 
				 chartArea2->AxisX->Interval = 100;
				 chartArea2->AxisX->Maximum = 1000;
				 chartArea2->AxisX->Minimum = -1000;
				 chartArea2->AxisY->Interval = 100;
				 chartArea2->AxisY->Maximum = 6000;
				 chartArea2->AxisY->Minimum = 0;
				 chartArea2->Name = L"ChartArea1";
				 this->chart2->ChartAreas->Add(chartArea2);
				 legend2->Name = L"Legend1";
				 this->chart2->Legends->Add(legend2);
				 this->chart2->Location = System::Drawing::Point(-61, -10);
				 this->chart2->Name = L"chart2";
				 series7->ChartArea = L"ChartArea1";
				 series7->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series7->Color = System::Drawing::Color::Khaki;
				 series7->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 18, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 series7->LabelForeColor = System::Drawing::Color::YellowGreen;
				 series7->Legend = L"Legend1";
				 series7->Name = L"Series_LiDAR";
				 series8->ChartArea = L"ChartArea1";
				 series8->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series8->Color = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)),
					 static_cast<System::Int32>(static_cast<System::Byte>(192)));
				 series8->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 series8->Legend = L"Legend1";
				 series8->MarkerColor = System::Drawing::Color::Blue;
				 series8->MarkerSize = 10;
				 series8->Name = L"Series_LiDAR_CLOSE";
				 series9->ChartArea = L"ChartArea1";
				 series9->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::FastPoint;
				 series9->Color = System::Drawing::Color::ForestGreen;
				 series9->Legend = L"Legend1";
				 series9->MarkerSize = 10;
				 series9->Name = L"Series_Radar_Angle";
				 series10->ChartArea = L"ChartArea1";
				 series10->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series10->Legend = L"Legend1";
				 series10->MarkerColor = System::Drawing::SystemColors::MenuHighlight;
				 series10->MarkerSize = 10;
				 series10->MarkerStyle = System::Windows::Forms::DataVisualization::Charting::MarkerStyle::Star4;
				 series10->Name = L"Series_TBox_LRadar";
				 series11->ChartArea = L"ChartArea1";
				 series11->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series11->Color = System::Drawing::Color::Black;
				 series11->Legend = L"Legend1";
				 series11->MarkerSize = 10;
				 series11->Name = L"Series_TBox_RRadar";
				 series12->ChartArea = L"ChartArea1";
				 series12->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series12->Legend = L"Legend1";
				 series12->MarkerSize = 10;
				 series12->Name = L"Series_RadarDetectArea";
				 this->chart2->Series->Add(series7);
				 this->chart2->Series->Add(series8);
				 this->chart2->Series->Add(series9);
				 this->chart2->Series->Add(series10);
				 this->chart2->Series->Add(series11);
				 this->chart2->Series->Add(series12);
				 this->chart2->Size = System::Drawing::Size(1389, 977);
				 this->chart2->TabIndex = 11;
				 this->chart2->Text = L"圖";
				 // 
				 // serialPort_LiDAR
				 // 
				 this->serialPort_LiDAR->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_LiDAR_DataReceived);
				 // 
				 // timer1
				 // 
				 this->timer1->Tick += gcnew System::EventHandler(this, &MyForm::timer1_Tick);
				 // 
				 // serialPort_Radar_Angle
				 // 
				 this->serialPort_Radar_Angle->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_Radar_Angle_DataReceived);
				 // 
				 // serialPort_Tbox
				 // 
				 this->serialPort_Tbox->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_Tbox_DataReceived);
				 // 
				 // timer2
				 // 
				 this->timer2->Tick += gcnew System::EventHandler(this, &MyForm::timer2_Tick);
				 // 
				 // MyForm
				 // 
				 this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
				 this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
				 this->AutoSizeMode = System::Windows::Forms::AutoSizeMode::GrowAndShrink;
				 this->ClientSize = System::Drawing::Size(1874, 961);
				 this->Controls->Add(this->tabControl1);
				 this->FormBorderStyle = System::Windows::Forms::FormBorderStyle::FixedSingle;
				 this->Name = L"MyForm";
				 this->StartPosition = System::Windows::Forms::FormStartPosition::CenterScreen;
				 this->Text = L"MyForm";
				 this->tabControl1->ResumeLayout(false);
				 this->tabPage1->ResumeLayout(false);
				 this->tabPage1->PerformLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart1))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
				 this->tabPage2->ResumeLayout(false);
				 this->groupBox1->ResumeLayout(false);
				 this->groupBox5->ResumeLayout(false);
				 this->groupBox5->PerformLayout();
				 this->groupBox8->ResumeLayout(false);
				 this->groupBox8->PerformLayout();
				 this->groupBox10->ResumeLayout(false);
				 this->groupBox10->PerformLayout();
				 this->groupBox7->ResumeLayout(false);
				 this->tabControl3->ResumeLayout(false);
				 this->tabPage5->ResumeLayout(false);
				 this->tabPage5->PerformLayout();
				 this->tabPage6->ResumeLayout(false);
				 this->tabPage6->PerformLayout();
				 this->groupBox9->ResumeLayout(false);
				 this->groupBox9->PerformLayout();
				 this->groupBox6->ResumeLayout(false);
				 this->groupBox6->PerformLayout();
				 this->tabControl2->ResumeLayout(false);
				 this->tabPage3->ResumeLayout(false);
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->Table_BSD))->EndInit();
				 this->tabPage4->ResumeLayout(false);
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->dataGridView1))->EndInit();
				 this->tabPage7->ResumeLayout(false);
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->dataGridView3))->EndInit();
				 this->tabPage8->ResumeLayout(false);
				 this->tabPage8->PerformLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart2))->EndInit();
				 this->ResumeLayout(false);

			 }
#pragma endregion
	private:System::String^ getRadarMode(int index)
	{
		switch (index)
		{
		case 0x01:
			return "BSD Mode";
			break;
		case 0x02:
			return  "RCTA Mode";
			break;
		case 0x03:
			return  "DOW Mode";
			break;
		}
	}
	private: System::Void Btn_Refresh_Combox_Click(System::Object^  sender, System::EventArgs^  e) {
		ComPortRefresh();
	}
	private:void ComPortRefresh(void)
	{
		cBox_LiDAR->Items->Clear();
		cBox_Radar_Angle->Items->Clear();
		cBox_TBox->Items->Clear();
		cBox_Radar->Items->Clear();
		cBox_CameraList->Items->Clear();

		cli::array<System::String^>^ Port = SerialPort::GetPortNames();
		cBox_LiDAR->Items->AddRange(Port);
		cBox_Radar_Angle->Items->AddRange(Port);
		cBox_TBox->Items->AddRange(Port);
		cBox_Radar->Items->AddRange(Port);
		std::map<int, Device> devices = de.getVideoDevicesMap();
		devices = de.getVideoDevicesMap();
		for (uint i = 0; i < devices.size(); i++)
		{
			System::String ^str = gcnew System::String(devices[i].deviceName.c_str());
			str = devices[i].id.ToString() + ":" + str;
			cBox_CameraList->Items->Add(str);
		}
	}
	private: System::Void Btn_CamCnt_Click(System::Object^  sender, System::EventArgs^  e) {
		if (cap.isOpened())cap.release();
		cap.open(Convert::ToInt16(cBox_CameraList->Text->Substring(0, 1)));

		ComPortNoRecord[2] = Convert::ToInt16(cBox_CameraList->Text->Substring(0, 1));

		string str = FileNameTime + (string)"\\VideoTest.avi";
		videoWrite.open(str, CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(640, 480));
		fstream fp_ComID;
		fp_ComID.open("ComRecord.txt", ios::out);
		fp_ComID << ComPortNoRecord[0] << " " << ComPortNoRecord[1] << " " << ComPortNoRecord[2] << endl;
		fp_ComID.close();
	}

	private: System::Void serialPort_LiDAR_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		cli::array<System::Byte>^ LiDAR_SerialPortData = gcnew cli::array<Byte>(10000);
		int ReadSize = serialPort_LiDAR->Read(LiDAR_SerialPortData, 0, 10000);

		for (int i = 0; i < ReadSize; i++)
		{
			if ((LiDAR_SerialPortData[i] == 0x02) && (LiDAR_SerialPortData[i + 1] == 0x80) && (LiDAR_SerialPortData[i + 4] == 0xB0) && (f_getHeader == false))
			{

				for (int j = i; j < ReadSize; j++)
				{
					LiDAR_Data[counter] = LiDAR_SerialPortData[j];
					counter++;
					if ((counter + 1) == 734)
						break;
				}
				f_getHeader = true;
				break;
			}
			if (f_getHeader)
			{
				if ((counter + 1) == 734)
					break;
				LiDAR_Data[counter] = LiDAR_SerialPortData[i];
				counter++;
			}
		}
		int Data[361];


		if ((counter + 1) == 734)
		{
			for (uint i = 0; i < 361; i++)
			{
				Data[i] = LiDAR_Data[2 * i + 7] + (LiDAR_Data[8 + i * 2] & 0x1F) * 256;
			}
			for (uint i = 0; i < 361; i++)
			{
				LIDAR_R_cooridate[i] = Data[i];
				LIDAR_X_cooridate[i] = (Data[i]) * cos((0.5 * i) * (M_PI / 180));
				LIDAR_Y_cooridate[i] = (Data[i]) * sin((0.5 * i) * (M_PI / 180));
			}
			f_getLiDARData = true;
			f_getHeader = false;
			counter = 0;
		}
	}
	private: System::Void Btn_LiDARCnt_Click(System::Object^  sender, System::EventArgs^  e) {
		if (serialPort_LiDAR->IsOpen)serialPort_LiDAR->Close();

		serialPort_LiDAR->PortName = cBox_LiDAR->Text;
		serialPort_LiDAR->Encoding = System::Text::Encoding::GetEncoding(28591);
		serialPort_LiDAR->BaudRate = 9600;
		serialPort_LiDAR->DataBits = 8;
		serialPort_LiDAR->StopBits = StopBits::One;
		serialPort_LiDAR->Parity = Parity::None;
		serialPort_LiDAR->Open();

		cli::array<System::Byte>^ LMS_Angular_range_change_manage = gcnew cli::array<Byte>{ 0x02, 0x00, 0x05, 0x00, 0x3B, 0xB4, 0x00, 0x32, 0x00, 0x3B, 0x1F };//更改LMS經度0.5度
		cli::array<System::Byte>^ continuous_LMS_data_manage = gcnew cli::array<Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x24, 0x34, 0x08 };//更改成連續指令緩區
		cli::array<System::Byte>^ LMS_baundrate_500k_manage = gcnew cli::array<Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x48, 0x58, 0x08 };//更改包率

		if (serialPort_LiDAR->IsOpen)
		{
			serialPort_LiDAR->Write(LMS_baundrate_500k_manage, 0, 8);
			Sleep(500);
			serialPort_LiDAR->Close();
			serialPort_LiDAR->BaudRate = 500000;
			serialPort_LiDAR->Open();
		}
		if (serialPort_LiDAR->IsOpen)
		{
			serialPort_LiDAR->Write(LMS_Angular_range_change_manage, 0, 11);
			_sleep(500);
			serialPort_LiDAR->Write(continuous_LMS_data_manage, 0, 8);
		}


	}
	private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
		
		if (cap.isOpened())
		{
			Mat frame1;
			cap >> frame1;
			videoWrite.write(frame1);
			ShowImage(pictureBox1, frame1);
		}
		chart1->Series["Series_LiDAR"]->Points->Clear();
		chart1->Series["Series_LiDAR_CLOSE"]->Points->Clear();
		chart1->Series["Series_Radar_Angle"]->Points->Clear();
		vector<Pt> Pt_newClusterRefPt;
		/*	if (!cBox_Record->Checked)
			{*/
		chart1->Series["Series_TBox_RRadar"]->Points->Clear();
		chart1->Series["Series_TBox_LRadar"]->Points->Clear();
		//}
#pragma region 光達
		if (serialPort_LiDAR->IsOpen)
		{

			vector<Pt>LIDAR_cooridate;
			LIDAR_cooridate.resize(0);
			LIDAR_cooridate.resize(361);
			fstream fp_Lidar; fp_Lidar.open(".\\" + FileNameTime + "\\Lidar.txt", ios::out | ios::app);

			for (uint i = 0; i < 361; i++)
			{
				LIDAR_cooridate[i] = Pt(LIDAR_X_cooridate[i], LIDAR_Y_cooridate[i]);
				LIDAR_cooridate[i].range = LIDAR_R_cooridate[i];
				fp_Lidar << LIDAR_R_cooridate[i] << " ";
			}
			fp_Lidar << t1 << " " << TBox.currentSpeed << endl;
			fp_Lidar.close();
			vector<int >lab;
			int nObj = DBSCAN(LIDAR_cooridate, 100.0,2);
			vector<vector<Pt>> Pt_ClusterList_new = Cluster2List(LIDAR_cooridate, nObj);
			Pt_newClusterRefPt.resize(Pt_ClusterList_new.size());
			int index = 0;
			for (uint16_t i = 0; i < Pt_ClusterList_new.size(); i++)
			{
				double  minX = 8000;
				double minY = 8000;
				Pt min;
				Color color = Color::FromArgb(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
				for (uint j = 0; j < Pt_ClusterList_new[i].size(); j++)
				{
					if (abs(Pt_ClusterList_new[i][j].x) < minX)
					{
						min.x = Pt_ClusterList_new[i][j].x;
						minX = abs(Pt_ClusterList_new[i][j].x);
					}
					if (abs(Pt_ClusterList_new[i][j].y) < minY)
					{
						minY = abs(Pt_ClusterList_new[i][j].y);
						min.y = Pt_ClusterList_new[i][j].y;
					}
					chart1->Series["Series_LiDAR"]->Points->AddXY(Pt_ClusterList_new[i][j].x, Pt_ClusterList_new[i][j].y);
					chart1->Series["Series_LiDAR"]->Points[index]->Color = color;
					index++;
				}
				Pt_newClusterRefPt[i] = min;
			}
			if (Pt_oldClusterRefPoint.size() == 0) {
				for (uint i = 0; i < Pt_newClusterRefPt.size(); i++)
					Pt_newClusterRefPt[i].KF_initial();
				Pt_oldClusterRefPoint = Pt_newClusterRefPt;
				return;
			}
			time_t t2 = clock();
			float time = (float)(t2 - t1) / CLK_TCK;
			t1 = t2;
			FindClosePoint(Pt_newClusterRefPt, Pt_oldClusterRefPoint, time, CurrentSpeed);
			for (uint i = 0; i < Pt_newClusterRefPt.size(); i++)
			{
				chart1->Series[1]->Points->AddXY(Pt_newClusterRefPt[i].x, Pt_newClusterRefPt[i].y);
				if (Pt_newClusterRefPt[i].velcity != 0)
					chart1->Series[1]->Points[i]->Label = "(" + Math::Round(Pt_newClusterRefPt[i].x, 2).ToString() + " , " + Math::Round(Pt_newClusterRefPt[i].y, 2).ToString() + " , " + Math::Round(Pt_newClusterRefPt[i].velcity / 100 * 3.6 + TBox.currentSpeed, 2).ToString() + ")";
			}
			chart1->Refresh();
			Pt_oldClusterRefPoint = Pt_newClusterRefPt;
		}
#pragma endregion

#pragma region 純角度的雷達
		if (serialPort_Radar_Angle->IsOpen)
		{
			lbBsdAngleT->Text = Math::Round(bsdAngle, 2).ToString();
			Pt Radar_Angle_Point;
			if (ckBox_RadarR->Checked)
			{
				Pt Rotation = CoordinateRotation(-125.0f, AngleRadar_Point);
				Radar_Angle_Point.x = Rotation.x + right_Radar_bias.x;
				Radar_Angle_Point.y = Rotation.y + right_Radar_bias.y;
			}
			else
			{
				Pt Rotationtmp = CoordinateRotation(-35.0f, AngleRadar_Point);
				Radar_Angle_Point.x = Rotationtmp.y + left_Radar_bias.x;
				Radar_Angle_Point.y = Rotationtmp.x + left_Radar_bias.y;
			}
			chart1->Series["Series_Radar_Angle"]->Points->AddXY(Radar_Angle_Point.x, Radar_Angle_Point.y);
		}
#pragma endregion

#pragma region TBox
		if (serialPort_Tbox->IsOpen)
		{

			if (std::abs(f_model_changed - TBox.L_RADAR_Mode) > 0)
			{
				//
				chart1->Series["Series_RadarDetectArea"]->Points->Clear();
				DrawBoundary(TBox.L_RADAR_Mode);
				chart1->Refresh();
				f_model_changed = TBox.L_RADAR_Mode;
				Tx_Radar_Mode->Text = "L:" + getRadarMode(TBox.L_RADAR_Mode) + "  R:" + getRadarMode(TBox.R_RADAR_Mode);
			}
			if (TBox.R_RADAR_ALert)
			{
				Pt R_RadarPtAtLiDAR = R_Radar2LiDAR(Pt(100 * TBox.R_RADAR_Range*Math::Cos(TBox.R_RADAR_Angle*M_PI / 180.f), 100 * TBox.R_RADAR_Range*Math::Sin(TBox.R_RADAR_Angle*M_PI / 180.f)));
				tx_TBox_RAngle->ForeColor = Color::Red;

				double minDistant = 8000000;
				double distant;
				Pt closetPt;
				for (uint i = 0; i < Pt_newClusterRefPt.size(); i++)
				{
					distant = sqrt(pow(Pt_newClusterRefPt[i].x - R_RadarPtAtLiDAR.x, 2) + pow(Pt_newClusterRefPt[i].y - R_RadarPtAtLiDAR.y, 2));
					if (distant < minDistant)
					{
						minDistant = distant;
						closetPt = Pt_newClusterRefPt[i];
					}
				}
			}
			else
				tx_TBox_RAngle->ForeColor = Color::Blue;

			if (TBox.L_RADAR_ALert)
			{

				tx_TBox_LAngle->ForeColor = Color::Red;
				tx_TBox_LAngle->Text = "L Range: " + TBox.L_RADAR_Range.ToString() + " L Angle: " + TBox.L_RADAR_Angle.ToString();
				Pt L_RadarPtAtLiDAR = L_Radar2LiDAR(Pt(100 * TBox.L_RADAR_Range*Math::Cos(TBox.L_RADAR_Angle*M_PI / 180.f), 100 * TBox.L_RADAR_Range*Math::Sin(TBox.L_RADAR_Angle*M_PI / 180.f)));
				double minDistant = 8000000;
				double distant;
				Pt closetPt;
				for (uint i = 0; i < Pt_newClusterRefPt.size(); i++)
				{
					distant = sqrt(pow(Pt_newClusterRefPt[i].x - L_RadarPtAtLiDAR.x, 2) + pow(Pt_newClusterRefPt[i].y - L_RadarPtAtLiDAR.y, 2));
					if (distant < minDistant)
					{
						minDistant = distant;
						closetPt = Pt_newClusterRefPt[i];
					}
				}
				chart1->Series["Series_TBox_LRadar"]->Points->AddXY(L_RadarPtAtLiDAR.x, L_RadarPtAtLiDAR.y);
			}
			else
				tx_TBox_LAngle->ForeColor = Color::Blue;
			Tx_CarSpeed->Text = TBox.currentSpeed.ToString();

			fstream fp_TBox;
			fp_TBox.open(".\\" + FileNameTime + "\\TBox.txt", ios::out | ios::app);
			fp_TBox << TBox.currentSpeed << " "
				<< TBox.L_RADAR_Mode << " "
				<< TBox.L_RADAR_ALert << " "
				<< TBox.L_RADAR_Range << " "
				<< TBox.L_RADAR_Speed << " "
				<< TBox.L_RADAR_Angle << " "
				<< TBox.R_RADAR_Mode << " "
				<< TBox.R_RADAR_ALert << " "
				<< TBox.R_RADAR_Range << " "
				<< TBox.R_RADAR_Speed << " "
				<< TBox.R_RADAR_Angle << " "
				<< System::DateTime::Now.Minute << " " << System::DateTime::Now.Second << endl;
			fp_TBox.close();
			tx_TBox_RAngle->Text = "R Range: " + TBox.R_RADAR_Range.ToString() + " R Angle: " + TBox.R_RADAR_Angle.ToString();


		}
#pragma endregion

#pragma region DB9雷達
		if (serialPort_Radar->IsOpen)
		{
			if (RadarData.ALert > 0)
			{
				fstream fptemp;
				char RadarFileName[100];
				int Date = System::DateTime::Now.Day * 10000 + System::DateTime::Now.Hour * 100 + System::DateTime::Now.Minute;
				sprintf(RadarFileName, "RadarData%d.txt", Date);
				fptemp.open(RadarFileName, ios::out | ios::app);
				fptemp << RadarData.Range << "\t" << RadarData.Angle << "\t" << RadarData.Speed << endl;
				fptemp.close();
				label9->Text = RadarData.Range.ToString() + " " + RadarData.Angle.ToString();
			}
		}
#pragma endregion	
		chart1->Refresh();
	}
	public:void ShowImage(System::Windows::Forms::PictureBox^ PBox, cv::Mat Image)
	{
		Mat image_Temp;
		switch (Image.type())
		{
		case CV_8UC3:
			Image.copyTo(image_Temp);

			break;
		case CV_8UC1:
			cvtColor(Image, image_Temp, CV_GRAY2RGB);
			break;
		default:
			break;
		}
		Bitmap ^ bmpimg = gcnew Bitmap(image_Temp.cols, image_Temp.rows, System::Drawing::Imaging::PixelFormat::Format24bppRgb);
		System::Drawing::Imaging::BitmapData^ data = bmpimg->LockBits(System::Drawing::Rectangle(0, 0, image_Temp.cols, image_Temp.rows), System::Drawing::Imaging::ImageLockMode::WriteOnly, System::Drawing::Imaging::PixelFormat::Format24bppRgb);
		Byte* dstData = reinterpret_cast<Byte*>(data->Scan0.ToPointer());

		unsigned char* srcData = image_Temp.data;

		for (int row = 0; row < data->Height; ++row)
		{
			memcpy(reinterpret_cast<void*>(&dstData[row*data->Stride]), reinterpret_cast<void*>(&srcData[row*image_Temp.step]), image_Temp.cols*image_Temp.channels());
		}

		bmpimg->UnlockBits(data);
		PBox->Image = bmpimg;
		PBox->SizeMode = PictureBoxSizeMode::StretchImage;
		PBox->Refresh();
		GC::Collect();
	}
	private: System::Void btn_RecordCnt_Click(System::Object^  sender, System::EventArgs^  e) {


		Btn_LiDARCnt->PerformClick();
		_sleep(500);
		Btn_TboxCnt->PerformClick();
		Btn_CamCnt->PerformClick();
	}
	private: System::Void Btn_Send_RadarAngle_Cmd_Click_1(System::Object^  sender, System::EventArgs^  e) {
		cli::array<byte>^ cmd = gcnew cli::array<System::Byte>{ 0x80, 0x64, 0xA1, 0x1F, 0x7C, 0x00, 0x00, 0xAB, 0xFD, 0x01, 0x00, 0xC9};
		serialPort_Radar_Angle->Write(cmd, 0, 12);
	}
	private: System::Void Btn_LiDARClose_Click(System::Object^  sender, System::EventArgs^  e) {
		cli::array<System::Byte>^ LMS_Stope_manage = gcnew cli::array<System::Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x25, 0x38, 0x08};
		serialPort_LiDAR->Write(LMS_Stope_manage, 0, 8);
		serialPort_LiDAR->Close();
	}
	private:void DrawBoundary(uint index)
	{
		switch (index)
		{
		case 1://BSD
		{
			for (uint j = 400; j < 1200; j += 100)
			{
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(380, j);
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(-380, j);


				chart2->Series["Series_RadarDetectArea"]->Points->AddXY(380, j);
				chart2->Series["Series_RadarDetectArea"]->Points->AddXY(-380, j);
			}
			for (int i = 0; i < 400; i += 100)
			{
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(i, 400);
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(-i, 400);
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(i, 1200);
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(-i, 1200);


				chart2->Series["Series_RadarDetectArea"]->Points->AddXY(i, 400);
				chart2->Series["Series_RadarDetectArea"]->Points->AddXY(-i, 400);
				chart2->Series["Series_RadarDetectArea"]->Points->AddXY(i, 1200);
				chart2->Series["Series_RadarDetectArea"]->Points->AddXY(-i, 1200);
			}

		}
		break;
		case 0x02://RCTA

			for (uint j = 0; j < 1000; j += 100)
			{
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(-2000, j);
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(2000, j);
			}
			for (int i = 0; i < 2000; i += 100)
			{
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(-i, 0);
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(-i, 1000);
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(i, 0);
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(i, 1000);
			}

			break;
		case 0x03://DOW
			for (uint j = 0; j < 2000; j += 100)
			{
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(380, j);
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(-380, j);
			}

			for (int i = 0; i < 380; i += 100)
			{
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(i, 0);
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(-i, 0);
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(i, 2000);
				chart1->Series["Series_RadarDetectArea"]->Points->AddXY(-i, 2000);
			}
			break;
		default:
			break;
		}

	}
	private: System::Void Btn_Radar_Connect_Click(System::Object^  sender, System::EventArgs^  e) {
		serialPort_Radar->PortName = cBox_Radar->Text;
		serialPort_Radar->BaudRate = 460800;
		serialPort_Radar->DataBits = 8;
		serialPort_Radar->StopBits = StopBits::One;
		serialPort_Radar->Parity = Parity::None;
		serialPort_Radar->Open();
	}
	private: System::Void Btn_TboxCnt_Click(System::Object^  sender, System::EventArgs^  e) {
		if (serialPort_Tbox->IsOpen)
		{
			serialPort_Tbox->Close();
			Sleep(10);
		}
		serialPort_Tbox->PortName = cBox_TBox->Text;
		serialPort_Tbox->BaudRate = 115200;
		serialPort_Tbox->DataBits = 8;
		serialPort_Tbox->StopBits = StopBits::One;
		serialPort_Tbox->Parity = Parity::None;
		serialPort_Tbox->Open();

		ComPortNoRecord[1] = Convert::ToInt16(cBox_TBox->Text->Remove(0, 3));

		fstream fp_ComID;
		fp_ComID.open("ComRecord.txt", ios::out);
		fp_ComID << ComPortNoRecord[0] << " " << ComPortNoRecord[1] << " " << ComPortNoRecord[2] << endl;
		fp_ComID.close();
	}

	private:bool _is_InRange(Pt P)
	{
		switch (TBox.L_RADAR_Mode)
		{
		case 0x01:
			if (abs(P.x) < 380 && P.y < 1200)//BSD
				return true;
			else
				return false;

			break;
		case 0x02:
			if (abs(P.x) < 200 && P.y < 100)//RCTA
				return true;
			else
				return false;
			break;
		case 0x03:
			if (abs(P.x) < 380 && P.y < 2000)//DOW
				return true;
			else
				return false;
			break;
		}
	}

	private: System::Void serialPort_Tbox_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		cli::array<System::Byte>^bTboxData = gcnew cli::array<Byte>(format);
		serialPort_Tbox->Read(bTboxData, 0, 1);

		if (bTboxData[0] == 0x80)
		{
			byte Checksum = 0;
			serialPort_Tbox->Read(bTboxData, 1, format - 1);
			for (uint i = 0; i < format - 1; i++)
			{
				Checksum += bTboxData[i];
			}
			if (Checksum == bTboxData[format - 1])
			{
				TBox.currentSpeed = bTboxData[9];
				TBox.L_RADAR_Mode = bTboxData[14];
				TBox.L_RADAR_ALert = bTboxData[15];
				TBox.L_RADAR_Range = bTboxData[16];
				TBox.L_RADAR_Speed = bTboxData[17] - 127;
				TBox.L_RADAR_Angle = bTboxData[18] - 127;
				TBox.R_RADAR_Mode = bTboxData[19];
				TBox.R_RADAR_ALert = bTboxData[20];
				TBox.R_RADAR_Range = bTboxData[21];
				TBox.R_RADAR_Speed = bTboxData[22] - 127;
				TBox.R_RADAR_Angle = bTboxData[23] - 127;
				CurrentSpeed = bTboxData[9];
			}
		}
		if (serialPort_Tbox->BytesToRead >= format * 2)
		{
			serialPort_Tbox->DiscardInBuffer();
		}
	}
	private: System::Void serialPort_Radar_Angle_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		cli::array<System::Byte>^ bufferBsd = gcnew cli::array<Byte>(12);
		byte bChecksum = 0;
		if (serialPort_Radar_Angle->BytesToRead < 12)
			return;
		if ((byte)serialPort_Radar_Angle->ReadByte() == 0x54)
		{
			bufferBsd[0] = 0x54;
			serialPort_Radar_Angle->Read(bufferBsd, 1, 11);
			for (int i = 0; i < 11; i++) // checksum store in end of Frame 
			{
				bChecksum += bufferBsd[i];
			}
			if (bChecksum != bufferBsd[11])
				return;
			if (bufferBsd[1] == 0x64)//判斷是不是回傳資料
			{
				byte check_send = bufferBsd[9];
				return;
			}
			//判斷Command有沒有送
			if (bufferBsd[2] != 0xA1)
			{
				//bsd_Messige("Not valid BSD data!\r\n", "error");
				SetLabelText("Command沒送");
				//Loading_BSD = false;
				return;
			}
			//判斷有無目標
			if ((bufferBsd[1] != 1) || (bufferBsd[4] != 0xff) || (bufferBsd[5] != 0xff) || (bufferBsd[6] != 0xff))
			{
				SetLabelText("沒有目標");
				//bsd_Messige("No goal!\r\n", "error");

				return;
			}
			bsdAngle = ((bufferBsd[9] + bufferBsd[10] * 256) - 10000) / 100.0;
			AngleRadar_Point = Pt(targetDistant*Math::Cos(bsdAngle*M_PI / 180.f), targetDistant*Math::Sin(bsdAngle*M_PI / 180.f));

		}
		if (serialPort_Radar_Angle->BytesToRead >= 24)
		{
			serialPort_Radar_Angle->DiscardInBuffer();// To Flush the BSD Data  
		}
	}
	protected: delegate void SetLabel(System::String^ str);
	private:void SetLabelText(System::String^ str)
	{
		if (this->lbBsdAngleT->InvokeRequired)
		{
			SetLabel ^d = gcnew SetLabel(this, &MyForm::SetLabelText);
			this->Invoke(d, gcnew cli::array<Object^> { str });
		}
		else
		{
			this->lbBsdAngleT->Text = str;
			this->lbBsdAngleT->Refresh();
		}
	}
	
	private:void LoadData()
	{
		std::fstream fp;
		fp.open("LBias.txt", std::ios::in);
		fp >> left_Radar_bias.x;
		fp >> left_Radar_bias.y;
		fp.close();
		fp.open("RBias.txt", std::ios::in);
		fp >> right_Radar_bias.x;
		fp >> right_Radar_bias.y;
		fp.close();
		tx_LRadarBias_X->Text = Math::Round(right_Radar_bias.x, 2).ToString();
		tx_LRadarBias_Y->Text = Math::Round(right_Radar_bias.y, 2).ToString();
		tx_RRadarBias_X->Text = Math::Round(left_Radar_bias.x, 2).ToString();
		tx_RRadarBias_Y->Text = Math::Round(left_Radar_bias.y, 2).ToString();
	}
	private: System::Void Btn_UpDateSetting_Click(System::Object^  sender, System::EventArgs^  e) {
		targetDistant = Convert::ToDouble(txBox_targetDistant->Text) * 100;
		PartitionValue = Convert::ToDouble(tBox_Partition->Text) * 100;
	}
	private: System::Void Btn_LeftBias_Click(System::Object^  sender, System::EventArgs^  e) {
		Pt Rotation = CoordinateRotation(-35.0f, AngleRadar_Point);
		double minDistant = 8000000;
		double distant;
		Pt closetPt;
		for (uint i = 0; i < Pt_oldClusterRefPoint.size(); i++)
		{
			distant = sqrt(pow(Pt_oldClusterRefPoint[i].x, 2) + pow(Pt_oldClusterRefPoint[i].y, 2));
			if (distant < minDistant)
			{
				minDistant = distant;
				left_Radar_bias.x = Pt_oldClusterRefPoint[i].x - Rotation.y;
				left_Radar_bias.y = Pt_oldClusterRefPoint[i].y - Rotation.x;
			}
		}


		tx_LRadarBias_X->Text = Math::Round(left_Radar_bias.x, 2).ToString();
		tx_LRadarBias_Y->Text = Math::Round(left_Radar_bias.y, 2).ToString();

		std::fstream fp;
		fp.open("LBias.txt", std::ios::out);
		fp << left_Radar_bias.x << " " << left_Radar_bias.y;
		fp.close();
		ckBox_RadarR->Checked = false;
	}
	private: System::Void Btn_RightBias_Click(System::Object^  sender, System::EventArgs^  e) {
		ckBox_RadarR->Checked = true;
		Pt Rotation = CoordinateRotation(-125.0f, AngleRadar_Point);
		double minDistant = 8000000;
		double distant;
		Pt closetPt;
		for (uint i = 0; i < Pt_oldClusterRefPoint.size(); i++)
		{
			distant = sqrt(pow(Pt_oldClusterRefPoint[i].x, 2) + pow(Pt_oldClusterRefPoint[i].y, 2));
			if (distant < minDistant)
			{
				minDistant = distant;
				right_Radar_bias.x = Pt_oldClusterRefPoint[i].x - Rotation.y;
				right_Radar_bias.y = Pt_oldClusterRefPoint[i].y - Rotation.x;
			}
		}
		tx_RRadarBias_X->Text = Math::Round(right_Radar_bias.x, 2).ToString();
		tx_RRadarBias_Y->Text = Math::Round(right_Radar_bias.y, 2).ToString();
		f_getRRadarBias = true;
		std::fstream fp;
		fp.open("RBias.txt", std::ios::out);
		fp << right_Radar_bias.x << " " << right_Radar_bias.y;
		fp.close();
	}
	private:Pt CoordinateRotation(double degree, Pt P)
	{
		Pt Ans;
		Ans.x = cos(degree*M_PI / 180)*P.x + sin(degree*M_PI / 180)*P.y;
		Ans.y = -sin(degree*M_PI / 180)*P.x + cos(degree*M_PI / 180)*P.y;
		return Ans;
	}
	private:Pt R_Radar2LiDAR(Pt P)
	{
		Pt Ans;
		Pt Rotation = CoordinateRotation(-125, P);
		Ans.x = Rotation.x + right_Radar_bias.x;
		Ans.y = Rotation.y + right_Radar_bias.y;
		return Ans;
	}
	private:Pt L_Radar2LiDAR(Pt P)
	{
		Pt Ans;
		Pt Rotationtmp = CoordinateRotation(-35.0f, P);
		Ans.x = Rotationtmp.y + left_Radar_bias.x;
		Ans.y = Rotationtmp.x + left_Radar_bias.y;
		return Ans;
	}
	private:void LoadComPort()
	{
		fstream fp_ComID;
		fp_ComID.open("ComRecord.txt", ios::in);
		fp_ComID >> ComPortNoRecord[0]; fp_ComID >> ComPortNoRecord[1]; fp_ComID >> ComPortNoRecord[2];
		fp_ComID.close();
		cBox_LiDAR->Text = "COM" + ComPortNoRecord[0].ToString();
		cBox_TBox->Text = "COM" + ComPortNoRecord[1].ToString();
		cBox_CameraList->Text = ComPortNoRecord[2].ToString();
	}
	private: System::Void Btn_PlayPause_Click(System::Object^  sender, System::EventArgs^  e) {
		timer2->Interval = 500;
		timer2->Start();
		fp_LiDarReader.open("Lidar.txt", ios::in);
		t1 = 0;
		DrawBoundary(1);
	}
	private: System::Void timer2_Tick(System::Object^  sender, System::EventArgs^  e) {

		char line[10000];
		chart2->Series["Series_LiDAR"]->Points->Clear();
		chart2->Series[1]->Points->Clear();
		fp_LiDarReader.getline(line, sizeof(line), '\n');
		System::String^ str = gcnew System::String(line);
		cli::array<System::String^> ^StringArray = str->Split(' ');
		vector<Pt>LIDAR_cooridate;
		if (StringArray->Length > 10)
		{
			for (uint i = 0; i < 361; i++)
			{
				LIDAR_X_cooridate[i] = System::Convert::ToDouble(StringArray[i]) * cos((0.5 * i) * (M_PI / 180));
				LIDAR_Y_cooridate[i] = System::Convert::ToDouble(StringArray[i]) * sin((0.5 * i) * (M_PI / 180));
				if (System::Convert::ToDouble(StringArray[i]) < 6000 && abs(LIDAR_X_cooridate[i]) < 1000)
					LIDAR_cooridate.push_back(Pt(LIDAR_X_cooridate[i], LIDAR_Y_cooridate[i], System::Convert::ToDouble(StringArray[i]), 0.5 * i));
			}
			CurrentSpeed = System::Convert::ToDouble(StringArray[362]);
			vector<Pt>Pt_newClusterRefPt;
			vector<int >lab;
			//int nObj = EuclidCluster(LIDAR_cooridate, 100);
			int nObj = DBSCAN(LIDAR_cooridate, 100.0, 2);
			vector<vector<Pt>> Pt_ClusterList_new = Cluster2List(LIDAR_cooridate, nObj);
			Pt_newClusterRefPt.resize(Pt_ClusterList_new.size());
			int index = 0;
			for (uint16_t i = 0; i < Pt_ClusterList_new.size(); i++)
			{
				double  minX = 8000;
				double minY = 8000;
				Pt min;
				Color color = Color::FromArgb(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
				for (uint j = 0; j < Pt_ClusterList_new[i].size(); j++)
				{
					if (Pt_ClusterList_new[i][j].isCore)
					{
						if (abs(Pt_ClusterList_new[i][j].x) < minX)
						{
							min.x = Pt_ClusterList_new[i][j].x;
							minX = abs(Pt_ClusterList_new[i][j].x);
						}
						if (abs(Pt_ClusterList_new[i][j].y) < minY)
						{
							minY = abs(Pt_ClusterList_new[i][j].y);
							min.y = Pt_ClusterList_new[i][j].y;
						}
					}
					chart2->Series["Series_LiDAR"]->Points->AddXY(Pt_ClusterList_new[i][j].x, Pt_ClusterList_new[i][j].y);
					chart2->Series["Series_LiDAR"]->Points[index]->Color = color;
					index++;
				}
				Pt_newClusterRefPt[i] = min;
				
			}
			if (Pt_oldClusterRefPoint.size() != Pt_newClusterRefPt.size()) {
				for (uint i = 0; i < Pt_newClusterRefPt.size(); i++)
					Pt_newClusterRefPt[i].KF_initial();
				Pt_oldClusterRefPoint = Pt_newClusterRefPt;
			    t1 = System::Convert::ToDouble(StringArray[361]);
				return;
			}

			time_t t2 = System::Convert::ToDouble(StringArray[361]);
			float time = (float)(t2 - t1) / CLK_TCK;
			t1 = t2;
			FindClosePoint(Pt_newClusterRefPt, Pt_oldClusterRefPoint, time, CurrentSpeed);
			for (uint i = 0; i < Pt_newClusterRefPt.size(); i++)
			{
				chart2->Series[1]->Points->AddXY(Pt_newClusterRefPt[i].x, Pt_newClusterRefPt[i].y);
				chart2->Series[1]->Points[i]->Label = "(" + Math::Round(Pt_newClusterRefPt[i].x, 2).ToString() + " , " + Math::Round(Pt_newClusterRefPt[i].y, 2).ToString() + " , " + Math::Round(Pt_newClusterRefPt[i].velcity - CurrentSpeed, 2).ToString() + ")";
			}
			Tx_CarSpeed2->Text = (-CurrentSpeed).ToString();
			chart2->Refresh();
			Pt_oldClusterRefPoint = Pt_newClusterRefPt;
		}
	}


private: System::Void cBox_LIDAR_Mode_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {
}
private: System::Void Btn_UpDateFileName_Click(System::Object^  sender, System::EventArgs^  e) {
	   
		char timeNow[30] = { 0 };
		uint currnetTime = System::DateTime::Now.Minute * 10000 + System::DateTime::Now.Second * 100 + System::DateTime::Now.Millisecond;
		sprintf(timeNow, "%d", currnetTime);
		FileNameTime = (string)"RecordData" + (string)timeNow;
		std::string str = (string)"mkdir " + FileNameTime;
		system(str.c_str());
		str = FileNameTime + (string)"\\VideoTest.avi";
		videoWrite.open(str, CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(640, 480));
}
};
}
