#!/usr/bin/env python

import gettext
import sys
import os
import subprocess
import psutil
import signal
import threading
import rospy
import wx

class MyFrame(wx.Frame):
	def __init__(self, *args, **kwds):
		kwds["style"] = wx.DEFAULT_FRAME_STYLE
		wx.Frame.__init__(self, *args, **kwds)

		self.id = '_'.join( cmd_out_lst('whoami ; hostname') )
		rospy.init_node( 'audio_comm_{}'.format(self.id) )

		self.clb = wx.CheckListBox(self)
		self.__set_properties()
		self.__do_layout()

		cmd = 'roslaunch audio_comm capture.launch topic:=audio_{}'.format(self.id)
		self.proc_cap = proc_exec(cmd)

		self.plays = {}
		self.ev = threading.Event()
		self.th = threading.Thread( target=self.th_func )
		self.th.daemon = True
		self.th.start()

		self.Bind(wx.EVT_CHECKLISTBOX, self.OnChecked, self.clb)
		self.Bind(wx.EVT_CLOSE, self.OnClose)

	def __set_properties(self):
		self.SetTitle(_("Audio Comm"))
		self.SetSize((240, 240))

	def __do_layout(self):
		sizer_1 = wx.BoxSizer(wx.VERTICAL)
		label = wx.StaticText( self, wx.ID_ANY, _(self.id) )
		sizer_1.Add(label, 0, wx.ALL, 4)
		sizer_1.Add(self.clb, 1, wx.EXPAND | wx.ALL, 4)
		self.SetSizer(sizer_1)
		self.Layout()

	def th_func(self):
		while not self.ev.wait(5.0):
			cmd_out_lst('echo -n y | rosnode cleanup')
			lst = cmd_out_lst('rosnode list | sed -n -e "s/\/*audio_comm_//p"')
			ids = sorted( filter( lambda s: s != self.id, lst ) )
			ks = sorted( self.plays.keys() )
			if ids != ks:
				for id in filter( lambda id: id not in ks, ids ):
					self.play_start(id)
				for id in filter( lambda id: id not in ids, ks ):
					self.play_stop(id, dic_del=True)
				wx.CallAfter( self.update_clb, ids )

	def update_clb(self, ids):
		self.clb.Set(ids)
		for (i, id) in enumerate(ids):
		        if self.plays.get(id):
				self.clb.Check(i)

	def OnClose(self, event):
		proc_term(self.proc_cap)
		for id in self.plays.keys():
			self.play_stop(id)
		self.ev.set()
		self.th.join()
		self.Destroy()

	def OnChecked(self, event):
		on_ids = self.clb.GetCheckedStrings()
		for id in self.plays.keys():
		        if id in on_ids:
				self.play_start(id)
			else:
				self.play_stop(id)

	def play_start(self, id):
	        if self.plays.get(id): # already started
			return
		cmd = 'roslaunch audio_comm play.launch topic:=audio_{}'.format(id)
		self.plays[id] = proc_exec(cmd)
		
	def play_stop(self, id, dic_del=False):
		proc_term( self.plays.get(id) )
		self.plays[id] = None
		if dic_del:
			del self.plays[id]

def cmd_out_lst(cmd):
	return subprocess.check_output(cmd, shell=True).strip().split(os.linesep)

def proc_exec(cmd):
	return psutil.Popen(cmd, shell=True)

def proc_term(proc):
	if not proc: # already terminated
		return
	f = proc.get_children if hasattr(proc, 'get_children') else proc.children
	procs = [ proc ] + f( recursive=True )
	for proc in reversed(procs):
		proc.send_signal(signal.SIGINT)
		proc.wait()

class MyApp(wx.App):
	def OnInit(self):
		wx.InitAllImageHandlers()
		frame_1 = MyFrame(None, wx.ID_ANY, "")
		self.SetTopWindow(frame_1)
		frame_1.Show()
		return 1

if __name__ == "__main__":
	gettext.install("app")
	app = MyApp(0)
	app.MainLoop()

# EOF
