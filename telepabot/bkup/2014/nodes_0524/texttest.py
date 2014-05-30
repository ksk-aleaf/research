#-*- coding: utf-8 -*-
import sys
from PyQt4.QtGui import *
from PyQt4.QtCore import *
#from PyQt4 import *


class HelloWindow(QMainWindow):
	def __init__(self, *args):
		QMainWindow.__init__(self, *args)
		base = QWidget(self)
		self.texthan10 = u"abcdefghij"
		self.textzen10 = u"あいうえおかきくけこ"
		self.texthan20 = u"abcdefghijklmnopqrst"
		#self.textzen20 = u"あいうえおかきくけこさしすせそたちつてと"
		self.textzen1 = u"小"
		self.textzen10 = u"小池京太郎小池京太郎"
		self.textzen20 = u"小池京太郎小池京太郎小池京太郎小池京太郎"
		#self.textzen20 = u"慶應慶應慶應慶應慶應慶應慶應慶應慶應慶應"
		self.setGeometry(0, 0, 200, 200)
	 
	def changed(self):
		self.tedit.append(self.ledit.text())

	def paintEvent(self, e):
		p = QPainter(self)
		pen = QPen(QColor.green)
		p.setPen(pen)
		p.setFont(QFont('Decorative',14))
		pf = QRectF()
		qs = QString(self.textzen1)
		qp = QPoint(300,300)
		bound = p.drawText(pf,1,qs)
		p.drawText(bound,1,qs)
		print "width:" + str(bound.width())
		print "height:" + str(bound.height())
		print "x:" + str(bound.x())
		print "y:" + str(bound.y())
		print "topLeft:"+str(bound.topLeft().x())+"  "+str(bound.topLeft().y())
		print "bottomRight:"+str(bound.bottomRight().x())+"  "+str(bound.bottomRight().y())
		
		listA = [1,2,3,4,5]
		listB = [6,7,8,9]
		listB = listA
		listA = [10,11,12]
		for num in listB:
			print num


		
def main(args):
	app = QApplication(args)
	win = HelloWindow()
	win.show()
	sys.exit(app.exec_())
		   
if __name__ == "__main__":
	main(sys.argv)
