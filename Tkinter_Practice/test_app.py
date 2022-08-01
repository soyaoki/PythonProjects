#!/usr/bin/env python
# -*- coding: utf8 -*-
import sys
import Tkinter

#
# ボタンが押されるとここが呼び出される
#
def Callback_Button(event):
  #エントリーの中身を削除
  EditBox.delete(0, Tkinter.END)

root = Tkinter.Tk()
root.title(u"Software Title")
root.geometry("400x300")

#エントリー
EditBox = Tkinter.Entry(width=50)
EditBox.insert(Tkinter.END,"挿入する文字列")
EditBox.pack()

#ボタン
Button = Tkinter.Button(text=u'ボタンです')
Button.bind("<Button-1>",Callback_Button)
#左クリック（<Button-1>）されると，DeleteEntryValue関数を呼び出すようにバインド
Button.pack()

#ここで，valueにEntryの中身が入る
value = EditBox.get()

root.mainloop()
