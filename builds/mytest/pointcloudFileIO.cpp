#include <sstream> 
#include <iostream>
#include <opencv2\opencv.hpp>
#include "pointcloudFileIO.h"

#include <fstream>
#include <iomanip>

int pointcloudFileIO::savePointCloud2DMatToRawTxt(std::string path, cv::Mat points,cv::Mat colors, cv::Mat isValids)
{
	//check
	if (path.empty())
	{
		std::cout << "path empty. save point cloud fail" << "\n";
		return ERR;
	}
	if (points.empty())
	{
		std::cout << "points empty. save point cloud fail" << "\n";
		return ERR;
	}
	if (colors.empty())
	{
		std::cout << "colors empty. save point cloud fail" << "\n";
		return ERR;
	}
	if (isValids.empty())
	{
		std::cout << "isValids empty. save point cloud fail" << "\n";
		return ERR;
	}

	if (points.size() != colors.size() || points.size() != isValids.size())
	{
		std::cout << "2D dimension inconsistent. save point cloud fail" << "\n";
		return ERR;
	}


	FILE *file;
	file = fopen(path.c_str(), "w");
	for (int h = 0; h < points.rows; h++)
	{
		for (int w = 0; w < points.cols; w++)
		{
			if (isValids.at<unsigned char>(h, w) == 1)
			{
				fprintf(file, "%f %f %f %d %d %d\n",
					points.at<cv::Vec3f>(h, w)[0], points.at<cv::Vec3f>(h, w)[1], points.at<cv::Vec3f>(h, w)[2],
					(int)colors.at<cv::Vec3b>(h, w)[0], (int)colors.at<cv::Vec3b>(h, w)[1], (int)colors.at<cv::Vec3b>(h, w)[2]);
			}
		}
	}
	fclose(file);
	return OK;
}

int pointcloudFileIO::savePointCloud2DMatToObj(std::string path, cv::Mat points, cv::Mat colors, cv::Mat isValids)
{
	//check
	if (path.empty())
	{
		std::cout << "path empty. save point cloud fail" << "\n";
		return ERR;
	}
	if (points.empty())
	{
		std::cout << "points empty. save point cloud fail" << "\n";
		return ERR;
	}
	if (colors.empty())
	{
		std::cout << "colors empty. save point cloud fail" << "\n";
		return ERR;
	}
	if (isValids.empty())
	{
		std::cout << "isValids empty. save point cloud fail" << "\n";
		return ERR;
	}

	if (points.size() != colors.size() || points.size() != isValids.size())
	{
		std::cout << "2D dimension inconsistent. save point cloud fail" << "\n";
		return ERR;
	}


	FILE *file;
	file = fopen(path.c_str(), "w");

	long int ptCount = 0;
	for (int h = 0; h < points.rows; h++)
	{
		for (int w = 0; w < points.cols; w++)
		{
			if (isValids.at<unsigned char>(h, w) == 1)
			{
				fprintf(file, "v %f %f %f %d %d %d\n",
					points.at<cv::Vec3f>(h, w)[0], points.at<cv::Vec3f>(h, w)[1], points.at<cv::Vec3f>(h, w)[2],
					(int)colors.at<cv::Vec3b>(h, w)[0], (int)colors.at<cv::Vec3b>(h, w)[1], (int)colors.at<cv::Vec3b>(h, w)[2]);
				ptCount++;
			}
		}
	}

	fprintf(file, "# %d vertices, 0 vertices normals\n\n", ptCount);
	fprintf(file, "# 0 faces, 0 coords texture\n\n");
	fprintf(file, "# End of File\n");


	fclose(file);
	return OK;





}
//Loading raw data from .txt file
int Load3DRawData(const char* cInName, cv::Mat &OutData)
{
	/***Fcn illustration***/
	//cInName = path name + file name
	//OutData = raw data from .txt file

	/******** Read row number ***********/
	int numOfRows = 0;
	{
		std::ifstream fin;
		fin.open(cInName, std::ios::in);
		if (!fin.is_open())
		{
			return -1;
		}

		while (!fin.eof())
		{
			std::string line;
			getline(fin, line);
			if (line.size() != 0)
			{
				numOfRows += 1;
			}
		}
		fin.close();
	}

	/******** Read data ***********/
	{
		std::ifstream fin;
		fin.open(cInName, std::ios::in);
		if (!fin.is_open())
		{
			return -1;
		}

		int iReadChCnt = -1;
		int iColNum = 0;
		int iRowNum = 0;
		//while whole file
		char cLine[255];

		OutData = cv::Mat::zeros(numOfRows, 3, CV_32F);
		while (!fin.eof() && iRowNum < OutData.rows)
		{
			// for each line
			iReadChCnt++;
			fin.get(cLine[iReadChCnt]);

			if (cLine[iReadChCnt] == ' ' || cLine[iReadChCnt] == '\t' || cLine[iReadChCnt] == '\n')
			{
				double dLine = atof(cLine);
				if (0 <= iColNum && iColNum <= 2)
				{
					OutData.at<float>(iRowNum, iColNum) = dLine;
					iColNum += 1;
				}

				if (cLine[iReadChCnt] == '\n')
				{
					iColNum = 0;
					iRowNum += 1;
				}

				memset(cLine, '\0', sizeof(cLine) / sizeof(char));
				iReadChCnt = -1;
			}
		}
		fin.close();
	}
	return 0;
}

int pointcloudFileIO::LoadPointCloudFromRawTxt(const std::string _PCDataPath, cv::Mat &PCData_)
{
	int iCheckStatus = 0;

	std::string PCDataPath = _PCDataPath;

	// check extension
	{
		char buffer[8];
		std::size_t length = PCDataPath.copy(buffer, 3, PCDataPath.size() - 3);
		buffer[length] = '\0';
		std::string strFileType = buffer;

		if (strFileType != "txt")
		{
			return -1;
		}
	}

	cv::Mat RawData;
	{
		char * path = (char *)malloc(PCDataPath.size() + 1);
		memcpy(path, PCDataPath.c_str(), PCDataPath.size() + 1);
		iCheckStatus = Load3DRawData(path, RawData);
		free(path);
	}

	PCData_ = RawData;

	return iCheckStatus;

}

int Get3DRawDataRowNum(const char* cInName, int *iOutRow);
int pointcloudFileIO::LoadPointCloudFromObj(const std::string _PCDataPath, cv::Mat &PCData_)
{
	const std::string filename = _PCDataPath;
	// check extension
	{
		char buffer[8];
		std::size_t length = filename.copy(buffer, 3, filename.size() - 3);
		buffer[length] = '\0';
		std::string strFileType = buffer;

		if (strFileType != "obj")
		{
			return ERR;
		}
	}
	// check row num
	int RowNum = -1;
	const char *c = _PCDataPath.c_str();
	Get3DRawDataRowNum(c, &RowNum);

	const int MAX_RAW_DATA_PT_NUM = 500000;
	if (RowNum > MAX_RAW_DATA_PT_NUM || RowNum < 1)
	{
		std::cerr << "Please limit the input data to be within " << MAX_RAW_DATA_PT_NUM << " points" << std::endl;
		return ERR;
	}

	PCData_ = cv::Mat::zeros(0, 3, CV_32F);

	//  load data
	FILE* fp;
	char strLine[300];
	if ((fp = fopen(filename.c_str(), "r")) == NULL)
	{
		std::cout << "Error while opening file" << std::endl;
		return ERR;
	}

	int CurRowNumber = 0;
	while (!feof(fp) && CurRowNumber < RowNum)
	{
		cv::Mat row = cv::Mat::zeros(1, 3, CV_32F);
		fgets(strLine, 300, fp);
		char a = ' ';
		float x = 0;
		float y = 0;
		float z = 0;
		sscanf(strLine, "%c %f %f %f", &a, &x, &y, &z);
		if (a == 'v')
		{
			row.at<float>(0) = x;
			row.at<float>(1) = y;
			row.at<float>(2) = z;
			PCData_.push_back(row);
		}
	}
	fclose(fp);
	return OK;
}

int pointcloudFileIO::LoadPointCloudWNormalFromObj(const std::string _PCDataPath, cv::Mat &PCData_, cv::Mat &NormalData_)
{
	const std::string filename = _PCDataPath;
	// check extension
	{
		char buffer[8];
		std::size_t length = filename.copy(buffer, 3, filename.size() - 3);
		buffer[length] = '\0';
		std::string strFileType = buffer;

		if (strFileType != "obj")
		{
			return ERR;
		}
	}
	// check row num
	int RowNum = -1;
	const char *c = _PCDataPath.c_str();
	Get3DRawDataRowNum(c, &RowNum);

	const int MAX_RAW_DATA_PT_NUM = 500000;
	if ( RowNum < 1)
	{
		std::cerr << "No points readed " << "\n";
		return ERR;
	}

	if (RowNum > MAX_RAW_DATA_PT_NUM)
	{
		std::cerr << "Please limit the input data to be within " << MAX_RAW_DATA_PT_NUM << " points" <<"\n";
		return ERR;
	}

	PCData_ = cv::Mat::zeros(RowNum, 3, CV_32F);
	NormalData_ = cv::Mat::zeros(RowNum, 3, CV_32F);

	//  load data
	FILE* fp;
	char strLine[300];
	if ((fp = fopen(filename.c_str(), "r")) == NULL)
	{
		std::cout << "Error while opening file" << std::endl;
		return ERR;
	}

	int CurRowNumber = 0;
	while (!feof(fp) && CurRowNumber < RowNum)
	{
		fgets(strLine, 300, fp);
		char a = ' ';
		float x = 0;
		float y = 0;
		float z = 0;
		float nx = 0;
		float ny = 0;
		float nz = 0;
		sscanf(strLine, "%c %f %f %f %f %f %f", &a, &x, &y, &z, &nx, &ny, &nz);
		if (a == 'v')
		{
			PCData_.at<float>(CurRowNumber,0) = x;
			PCData_.at<float>(CurRowNumber,1) = y;
			PCData_.at<float>(CurRowNumber,2) = z;
			NormalData_.at<float>(CurRowNumber,0) = nx;
			NormalData_.at<float>(CurRowNumber,1) = ny;
			NormalData_.at<float>(CurRowNumber,2) = nz;
		}
		CurRowNumber++;
	}
	fclose(fp);
	return OK;
}

// from opencv 
static std::vector<std::string> split(const std::string &text, char sep) 
{
	std::vector<std::string> tokens;
	std::size_t start = 0, end = 0;
	while ((end = text.find(sep, start)) != std::string::npos)
	{
		tokens.push_back(text.substr(start, end - start));
		start = end + 1;
	}
	tokens.push_back(text.substr(start));
	return tokens;
}

int pointcloudFileIO::LoadPointCloudWNormalFromPly(const std::string _PCDataPath, cv::Mat &PCData_, cv::Mat &NormalData_)
{
	cv::Mat cloud;
	int numVertices = 0;
	int numCols = 3;
	int has_normals = 0;

	std::ifstream ifs(_PCDataPath);

	if (!ifs.is_open())
	{
		std::cout << std::string("Error opening input file: ") + std::string(_PCDataPath) + "\n";
	}

	std::string str;
	while (str.substr(0, 10) != "end_header")
	{
		std::vector<std::string> tokens = split(str, ' ');
		if (tokens.size() == 3)
		{
			if (tokens[0] == "element" && tokens[1] == "vertex")
			{
				numVertices = atoi(tokens[2].c_str());
			}
			else if (tokens[0] == "property")
			{
				if (tokens[2] == "nx" || tokens[2] == "normal_x")
				{
					has_normals = -1;
					numCols += 3;
				}
				else if (tokens[2] == "r" || tokens[2] == "red")
				{
					//has_color = true;
					numCols += 3;
				}
				else if (tokens[2] == "a" || tokens[2] == "alpha")
				{
					//has_alpha = true;
					numCols += 1;
				}
			}
		}
		else if (tokens.size() > 1 && tokens[0] == "format" && tokens[1] != "ascii")
		{
			std::cout << "Cannot read file, only ascii ply format is currently supported..." << "\n";
		}
		std::getline(ifs, str);
	}


	bool withNormals = true;

	cloud = cv::Mat(numVertices, withNormals ? 6 : 3, CV_32FC1);

	for (int i = 0; i < numVertices; i++)
	{
		float* data = cloud.ptr<float>(i);
		int col = 0;
		for (; col < (withNormals ? 6 : 3); ++col)
		{
			ifs >> data[col];
		}
		for (; col < numCols; ++col)
		{
			float tmp;
			ifs >> tmp;
		}
		/*
		if (withNormals)
		{
			// normalize to unit norm
			double norm = sqrt(data[3] * data[3] + data[4] * data[4] + data[5] * data[5]);
			if (norm>0.00001)
			{
				data[3] /= static_cast<float>(norm);
				data[4] /= static_cast<float>(norm);
				data[5] /= static_cast<float>(norm);
			}
		}
		*/
	}


	PCData_ = cv::Mat(numVertices, 3, CV_32FC1);
	NormalData_ = cv::Mat(numVertices, 3, CV_32FC1);

	cv::Mat tmpPC = cv::Mat(3, numVertices, CV_32FC1);
	cv::Mat tmpNormal = cv::Mat(3, numVertices, CV_32FC1);
	cv::Mat tmpData = cloud.t();

	tmpData.row(0).copyTo(tmpPC.row(0));
	tmpData.row(1).copyTo(tmpPC.row(1));
	tmpData.row(2).copyTo(tmpPC.row(2));

	tmpData.row(3).copyTo(tmpNormal.row(0));
	tmpData.row(4).copyTo(tmpNormal.row(1));
	tmpData.row(5).copyTo(tmpNormal.row(2));
	/*
	for (int i = 0; i < 3; ++i)
	{

		std::cout << tmpPC.at<float>(0, i) << " " << tmpPC.at<float>(1, i) << " " << tmpPC.at<float>(2, i) << "\n";
		std::cout << tmpNormal.at<float>(0, i) << " " << tmpNormal.at<float>(1, i) << " " << tmpNormal.at<float>(2, i) << "\n";
		std::cout << "\n";

	
	}
	*/
	PCData_ = tmpPC.t();
	NormalData_ = tmpNormal.t();

	return 0;
}

int pointcloudFileIO::ScalePointCloud(cv::Mat &PCData, float scaling_factor)
{
	PCData.convertTo(PCData, CV_32F);
	for (int i = 0; i < PCData.rows; ++i)
	{
		PCData.row(i).at<float>(0) *= scaling_factor;
		PCData.row(i).at<float>(1) *= scaling_factor;
		PCData.row(i).at<float>(2) *= scaling_factor;
	}
	return OK;
}

int pointcloudFileIO::SavePointCloudToObj(const std::string saveToPath, const cv::Mat &points, int iMatDataType)
{
	//#define CV_8U   0
	//#define CV_8S   1
	//#define CV_16U  2
	//#define CV_16S  3
	//#define CV_32S  4
	//#define CV_32F  5
	//#define CV_64F  6
	//#define CV_USRTYPE1 7
	if (points.empty())
	{
		std::cout << "no points\n";
		return -1;
	}
	if (saveToPath.empty())
	{
		std::cout << "path empty\n";
		return -1;
	}

	std::ofstream fout(saveToPath.c_str());
	for (int i = 0; i < points.rows; i++)
	{
		fout << "v" << ' ';
		for (int j = 0; j < points.cols; j++)
		{
			if (iMatDataType == 0)
			{
				if (j < points.cols - 1)
				{
					fout << points.at<unsigned char>(i, j) << ' ';
				}
				else
				{
					fout << points.at<unsigned char>(i, j);
				}
			}
			else if (iMatDataType == 4)
			{
				if (j < points.cols - 1)
				{
					fout << points.at<int>(i, j) << ' ';
				}
				else
				{
					fout << points.at<int>(i, j);
				}
			}
			else if (iMatDataType == 5)
			{
				if (j < points.cols - 1)
				{
					fout << points.at<float>(i, j) << ' ';
				}
				else
				{
					fout << points.at<float>(i, j);
				}
			}
			else if (iMatDataType == 6)
			{
				if (j < points.cols - 1)
				{
					fout << std::setprecision(45) << points.at<double>(i, j) << ' ';
				}
				else
				{
					fout << std::setprecision(45) << points.at<double>(i, j);
				}
			}
		}
		if (i < points.rows - 1)
		{
			fout << '\n';
		}
	}
	fout.close();
	return 0;
}
int pointcloudFileIO::SavePointCloudToRawTxt(const std::string saveToPath, const cv::Mat &points, int iMatDataType)
{
	//#define CV_8U   0
	//#define CV_8S   1
	//#define CV_16U  2
	//#define CV_16S  3
	//#define CV_32S  4
	//#define CV_32F  5
	//#define CV_64F  6
	//#define CV_USRTYPE1 7
	if (points.empty())
	{
		std::cout << "no points\n";
		return -1;
	} 
	if (saveToPath.empty())
	{
		std::cout << "path empty\n";
		return -1;
	}
	
	std::ofstream fout(saveToPath.c_str());
	for (int i = 0; i < points.rows; i++)
	{
		for (int j = 0; j < points.cols; j++)
		{
			if (iMatDataType == 0)
			{
				if (j < points.cols - 1)
				{
					fout << points.at<unsigned char>(i, j) << '\t';
				}
				else
				{
					fout << points.at<unsigned char>(i, j);
				}
			}
			else if (iMatDataType == 4)
			{
				if (j < points.cols - 1)
				{
					fout << points.at<int>(i, j) << '\t';
				}
				else
				{
					fout << points.at<int>(i, j);
				}
			}
			else if (iMatDataType == 5)
			{
				if (j < points.cols - 1)
				{
					fout << points.at<float>(i, j) << '\t';
				}
				else
				{
					fout << points.at<float>(i, j);
				}
			}
			else if (iMatDataType == 6)
			{
				if (j < points.cols - 1)
				{
					fout << std::setprecision(45) << points.at<double>(i, j) << '\t';
				}
				else
				{
					fout << std::setprecision(45) << points.at<double>(i, j);
				}
			}
		}
		if (i < points.rows - 1)
		{
			fout << '\n';
		}
	}
	fout.close();
	return 0;
}


int Get3DRawDataRowNum(const char* cInName, int *iOutRow)
{
	/***Fcn illustration***/
	//cInName = path name + file name
	//iOutRow = number of rows of raw data
	/***Fcn illustration***/
	std::string line;
	std::ifstream fin;
	fin.open(cInName, std::ios::in);
	if (!fin.is_open())
	{
		return -1;
	}

	*iOutRow = 0;
	while (!fin.eof())
	{
		getline(fin, line);
		if (line.size() != 0)
			*iOutRow += 1;
	}
	fin.close();
	return 0;
}

int pointcloudFileIO::SavePointCloudWNormalToPly(
	const std::string _PCDataPath, 
	const cv::Mat &PCData_, 
	const cv::Mat &NormalData_)
{

	if (PCData_.rows != NormalData_.rows)
	{
		std::cout << "data size mismatch!SavePointCloudWNormalToPly fail.\n";
		return -1;
	}

	std::FILE* f = std::fopen(_PCDataPath.c_str(), "w");
	if (f == NULL)
	{
		return -1;
	}

	//Headers
	std::fprintf(f, "ply\n");
	std::fprintf(f, "format ascii 1.0\n");
	std::fprintf(f, "comment \n");
	std::fprintf(f, "element vertex %d \n", PCData_.rows);
	std::fprintf(f, "property float x\n");
	std::fprintf(f, "property float y\n");
	std::fprintf(f, "property float z\n");
	std::fprintf(f, "property float nx\n");
	std::fprintf(f, "property float ny\n");
	std::fprintf(f, "property float nz\n");
	std::fprintf(f, "element face 0\n");
	std::fprintf(f, "property list uchar int vertex_indices\n");
	std::fprintf(f, "end_header\n");
	
	//Data
	for (int i = 0; i < PCData_.rows; ++i)
	{
		std::fprintf(f, "%f %f %f %f %f %f\n",
			PCData_.at<float>(i, 0),
			PCData_.at<float>(i, 1),
			PCData_.at<float>(i, 2),
			NormalData_.at<float>(i, 0),
			NormalData_.at<float>(i, 1),
			NormalData_.at<float>(i, 2));
	}

	std::fclose(f);
	return 0;
}