# opencv
rm考核
装甲板识别做得准确率一般。有误判，漏判，数字识别还未实现，计划使用svm识别，使用了20年吉林大学开源的训练好的svm模型，但传入进行predit时报错为(-215:Assertion failed) samples.cols == var_count && samples.type() == CV_32F in function 'predict'，多次尝试转化格式均未成功，问题还没能解决
