// saves all the images of the dictionary indicated to a dicrectory
/**
Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
	  conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
	  of conditions and the following disclaimer in the documentation and/or other materials
	  provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
*/


#include "dictionary.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <clipp.h>

using namespace std;

int main(int argc, char** argv)
{
	try
	{
		int pixSize = 75;
		string outdir;
		string dictionaryString;
		bool help = false;
		bool enclosed_corners = false;
		{
			using namespace clipp;//https://github.com/muellan/clipp
			auto cli = ((
				value("outdir", outdir).doc("set output directory (it must exist)"),
				value("dict", dictionaryString).doc("set dictionary"),
				option("-s").doc("set bit_image_size: 75 default") & value("bit_image_size", pixSize),
				option("-e").doc("set enclosed_corners for subpixel refinement").set(enclosed_corners,true)
				) | option("-h", "--help").doc("show help").set(help, true)
				);
			if (!parse(argc, argv, cli) || help) {
				cout << make_man_page(cli, argv[0]);
				cerr << "Dictionaries: ";
				for (auto dict : aruco::Dictionary::getDicTypes())
					cerr << dict << " ";
				return 0;
			}
		}
		aruco::Dictionary dict = aruco::Dictionary::load(dictionaryString);
		string dict_name = dict.getName();
		std::transform(dict_name.begin(), dict_name.end(), dict_name.begin(), ::tolower);
		//
		for (auto m : dict.getMapCode())
		{
			string number = std::to_string(m.second);
			while (number.size() != 5)
				number = "0" + number;
			stringstream name;
			name << argv[1] << "/" + dict_name + "_" << number << ".png";
			cout << name.str() << endl;
			cv::imwrite(name.str(), dict.getMarkerImage_id(m.second, pixSize, true, enclosed_corners, true, true));
		}
	}
	catch (std::exception& ex)
	{
		cerr << ex.what() << endl;
	}
}