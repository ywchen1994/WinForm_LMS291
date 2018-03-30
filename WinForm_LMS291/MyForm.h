#pragma once
#define _USE_MATH_DEFINES
#include <Windows.h>
#include"math.h"
#include<vector>
typedef unsigned int uint;
namespace WinForm_LMS291 {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO;
	using namespace System::IO::Ports;
	using namespace std;
	int LiDAR_Data[722];
	double LIDAR_X_cooridate[361];
	double LIDAR_Y_cooridate[361];

	/// <summary>
	/// MyForm 的摘要
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO:  在此加入建構函式程式碼
			//
			ComPortRefresh();
			timer1->Interval = 20;
			timer1->Start();
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
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  chart1;
	protected:
	private: System::IO::Ports::SerialPort^  serialPort_LiDAR;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::ComboBox^  cBox_LiDAR;
	private: System::Windows::Forms::Button^  Btn_LiDAR_Connected;
	private: System::Windows::Forms::Button^  Btn_LiDAR_DisConnect;
	private: System::Windows::Forms::Button^  Btn_Refresh_Combox;
	private: System::ComponentModel::IContainer^  components;

	private:
		/// <summary>
		/// 設計工具所需的變數。
		/// </summary>
		uint counter = 0;
		bool f_getLiDARData = false;
	private: System::Windows::Forms::Timer^  timer1;
			 bool f_getHeader = false;

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
			this->chart1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->serialPort_LiDAR = (gcnew System::IO::Ports::SerialPort(this->components));
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->cBox_LiDAR = (gcnew System::Windows::Forms::ComboBox());
			this->Btn_LiDAR_Connected = (gcnew System::Windows::Forms::Button());
			this->Btn_LiDAR_DisConnect = (gcnew System::Windows::Forms::Button());
			this->Btn_Refresh_Combox = (gcnew System::Windows::Forms::Button());
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart1))->BeginInit();
			this->groupBox1->SuspendLayout();
			this->SuspendLayout();
			// 
			// chart1
			// 
			chartArea1->AxisX->Interval = 100;
			chartArea1->AxisX->Maximum = 2000;
			chartArea1->AxisX->Minimum = -2000;
			chartArea1->AxisY->Interval = 50;
			chartArea1->AxisY->Maximum = 2000;
			chartArea1->AxisY->Minimum = 0;
			chartArea1->Name = L"ChartArea1";
			this->chart1->ChartAreas->Add(chartArea1);
			legend1->Name = L"Legend1";
			this->chart1->Legends->Add(legend1);
			this->chart1->Location = System::Drawing::Point(1, 12);
			this->chart1->Name = L"chart1";
			series1->ChartArea = L"ChartArea1";
			series1->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::FastPoint;
			series1->Legend = L"Legend1";
			series1->Name = L"Series_LiDAR";
			this->chart1->Series->Add(series1);
			this->chart1->Size = System::Drawing::Size(1257, 666);
			this->chart1->TabIndex = 0;
			this->chart1->TabStop = false;
			this->chart1->Text = L"chart1";
			// 
			// serialPort_LiDAR
			// 
			this->serialPort_LiDAR->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_LiDAR_DataReceived);
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->cBox_LiDAR);
			this->groupBox1->Controls->Add(this->Btn_LiDAR_Connected);
			this->groupBox1->Controls->Add(this->Btn_LiDAR_DisConnect);
			this->groupBox1->Location = System::Drawing::Point(1067, 79);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(182, 100);
			this->groupBox1->TabIndex = 2;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"LiDAR";
			// 
			// cBox_LiDAR
			// 
			this->cBox_LiDAR->FormattingEnabled = true;
			this->cBox_LiDAR->Location = System::Drawing::Point(8, 41);
			this->cBox_LiDAR->Name = L"cBox_LiDAR";
			this->cBox_LiDAR->Size = System::Drawing::Size(87, 20);
			this->cBox_LiDAR->TabIndex = 0;
			// 
			// Btn_LiDAR_Connected
			// 
			this->Btn_LiDAR_Connected->Location = System::Drawing::Point(101, 21);
			this->Btn_LiDAR_Connected->Name = L"Btn_LiDAR_Connected";
			this->Btn_LiDAR_Connected->Size = System::Drawing::Size(75, 23);
			this->Btn_LiDAR_Connected->TabIndex = 5;
			this->Btn_LiDAR_Connected->Text = L"連接";
			this->Btn_LiDAR_Connected->UseVisualStyleBackColor = true;
			this->Btn_LiDAR_Connected->Click += gcnew System::EventHandler(this, &MyForm::Btn_LiDAR_Connected_Click);
			// 
			// Btn_LiDAR_DisConnect
			// 
			this->Btn_LiDAR_DisConnect->Location = System::Drawing::Point(101, 61);
			this->Btn_LiDAR_DisConnect->Name = L"Btn_LiDAR_DisConnect";
			this->Btn_LiDAR_DisConnect->Size = System::Drawing::Size(75, 23);
			this->Btn_LiDAR_DisConnect->TabIndex = 6;
			this->Btn_LiDAR_DisConnect->Text = L"關閉";
			this->Btn_LiDAR_DisConnect->UseVisualStyleBackColor = true;
			this->Btn_LiDAR_DisConnect->Click += gcnew System::EventHandler(this, &MyForm::Btn_LiDAR_DisConnect_Click);
			// 
			// Btn_Refresh_Combox
			// 
			this->Btn_Refresh_Combox->Location = System::Drawing::Point(1075, 201);
			this->Btn_Refresh_Combox->Name = L"Btn_Refresh_Combox";
			this->Btn_Refresh_Combox->Size = System::Drawing::Size(75, 23);
			this->Btn_Refresh_Combox->TabIndex = 3;
			this->Btn_Refresh_Combox->Text = L"更新列表";
			this->Btn_Refresh_Combox->UseVisualStyleBackColor = true;
			this->Btn_Refresh_Combox->Click += gcnew System::EventHandler(this, &MyForm::Btn_Refresh_Combox_Click);
			// 
			// timer1
			// 
			this->timer1->Tick += gcnew System::EventHandler(this, &MyForm::timer1_Tick);
			// 
			// MyForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1270, 705);
			this->Controls->Add(this->Btn_Refresh_Combox);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->chart1);
			this->Name = L"MyForm";
			this->Text = L"MyForm";
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart1))->EndInit();
			this->groupBox1->ResumeLayout(false);
			this->ResumeLayout(false);

		}
#pragma endregion
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
				LIDAR_X_cooridate[i] = (Data[i]) * cos((0.5 * i) * (M_PI / 180));
				LIDAR_Y_cooridate[i] = (Data[i]) * sin((0.5 * i) * (M_PI / 180));
			}
			f_getLiDARData = true;
			f_getHeader = false;
			counter = 0;
		}
	}
	private: System::Void Btn_LiDAR_Connected_Click(System::Object^  sender, System::EventArgs^  e) {
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
	private: System::Void Btn_Refresh_Combox_Click(System::Object^  sender, System::EventArgs^  e) {
		ComPortRefresh();
	}
	private:void ComPortRefresh(void)
	{
		cBox_LiDAR->Items->Clear();
		cli::array<System::String^>^ Port = SerialPort::GetPortNames();
		cBox_LiDAR->Items->AddRange(Port);
	}
	private: System::Void Btn_LiDAR_DisConnect_Click(System::Object^  sender, System::EventArgs^  e) {
		timer1->Stop();
		cli::array<System::Byte>^ LMS_Stope_manage = gcnew cli::array<System::Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x25, 0x38, 0x08};
		serialPort_LiDAR->Write(LMS_Stope_manage, 0, 8);
		serialPort_LiDAR->Close();
	}
private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
	chart1->Series["Series_LiDAR"]->Points->Clear();
	if (f_getLiDARData)
	{
	 
		for (uint i = 0; i < 361; i++)
		{
			chart1->Series["Series_LiDAR"]->Points->AddXY(LIDAR_X_cooridate[i], LIDAR_Y_cooridate[i]);
		}
	}

}
};
}
