#!/usr/bin/env python
# -*- coding: utf8 -*-

try:
    import Tkinter as tk
except:
    import tkinter as tk

import time

class App():
    def __init__(self):
        self.root = tk.Tk()
        self.i = 0
        self.txt = ["◯", "×", "△", "□"]
        #ラベル
        self.label = tk.Label(text="", font=('Helvetica', 48), fg='black')
        self.label.pack()

        #入力
        self.editbox = tk.Entry(width=50)
        self.editbox.insert(tk.END,"Write answer here.")
        self.editbox.pack()

        #ボタン
        self.button = tk.Button(self.root, text="delete answer.", command=self.callback_button)
        self.button.pack()

        self.next_flg = tk.IntVar()
        self.next_button = tk.Button(self.root, text="Next", command=lambda: self.next_flg.set(1))
        self.next_button.pack()

        self.update()
        self.root.mainloop()

    def update(self):
        self.label.configure(text=self.txt[self.i])
        if(self.i + 1 < len(self.txt)):
            self.i = self.i + 1
        else:
            self.i = 0
        self.next_button.wait_variable(self.next_flg)
        self.root.after(5000, self.update) # msecで指定

    def callback_button(self):
        with open("test.txt", mode='w') as f:
            f.write(self.editbox.get())
        self.editbox.delete(0, tk.END)

app=App()
