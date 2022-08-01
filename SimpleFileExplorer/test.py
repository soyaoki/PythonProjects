import PySimpleGUI as sg
import glob
import os
import subprocess

def Read():
    global URL
    filelist = []
    for f in glob.glob(URL+"\*"):
        filelist.append(os.path.split(f)[1])
    return filelist
    
def Open():
    global URL
    global selected_file
    
    subprocess.Popen(['start', URL], shell=True)
    if selected_file != []:
        subprocess.Popen(['start', URL+'\\'+selected_file[0]], shell=True)
    return 0

def DoubleClick():
    global filelist
    global URL
    global selected_file
    if selected_file != []:
        URL = URL+'\\'+selected_file[0]
        filelist = Read()
    Refresh()
    return 0

def Refresh():
    global window
    global filelist
    global URL
    global reflesh_need_flag
    window['-FileName-'].update(values=filelist)
    window['-URL-'].update(URL)
    return 0

selected_file = 'None'
filelist = []
URL = r'C:\Users\soya0\Desktop'
reflesh_need_flag = False

window = None

def main():
    sg.theme('Python')
    
    global window
    global filelist
    global URL
    global selected_file
    global reflesh_need_flag
    
    filelist = Read()
    
    list_box_style = {
        'values': filelist,
        'select_mode': 'LISTBOX_SELECT_MODE_SINGLE',
        'enable_events': True,
        'bind_return_key': True,
        'size': (100, 20),
        'key': '-FileName-',
    }
    
    layout = [ 
        [sg.Text('簡易ファイルエクスプローラ（以下のテキストボックスにURLを入力して Enter でも使えます。）', tooltip='こんなこともできるよ')],
        [sg.InputText(URL, key='-URL-', size=(100, 10))],
        [sg.Button('読み込む', key='-Read-'), sg.Button('1つ上のフォルダへ', key='-Up-'), sg.Button('開く', key='-Open-')], 
        [sg.Listbox(**list_box_style)],
        [sg.Button('終了', key='-Exit-')]
    ]
    
    window = sg.Window('Test', layout, finalize=True)
    window['-FileName-'].bind('<Double-Button-1>', '+DoubleClick+')
    window['-URL-'].bind('<Enter>', '+MOUSE OVER+')
    window['-URL-'].bind("<Return>", "+Enter+")
    
    while True:
        
        event, values = window.read(timeout=250)
        selected_file = values['-FileName-']
        
        if event != '__TIMEOUT__':
            print(event)
        
        if event in (sg.WIN_CLOSED, '-Exit-'):
            print('exit')
            break
        
        elif event == '-Read-':
            filelist = Read()
            Refresh()
            
        elif event == '-Up-':
            URL = os.path.split(URL)[0]
            filelist = Read()
            Refresh()
            
        elif event == '-FileName-'+'+DoubleClick+':
            DoubleClick()
            
        elif event == '-URL-'+'+Enter+':
            URL = values['-URL-']
            filelist = Read()
            Refresh()
        
        elif event == '-Open-':
            Open()
    
    window.close()
    return 0
    

if __name__ == '__main__':
    main()
    