# GUI SSH
This module was designed to help the user making custom buttons on a GUI which will create the opportunity to run terminal commands easily with buttons clicked.
## Requirements
- Python 3
- screen module
- ROS
To install python module requirements use the following command:
```python3 -m pip install -r requirements```
To install screen use the following command:
```sudo apt install screen```
## Creating Custom Buttons
To create your own buttons, you just need to change the *buttons.json* file in the *ReadFiles* folder. You need to give the following data of your button:
- id: the unique ID of the button, which will be the variable name in the script
- textColor: the color of the text. You can make you own custom color using rgb(red value, green value, blue value). Given *rgb(0, 255, 255)* for example is equal to *cyan* color.
- bgColor: the color of the text background. Works the same as textColor.
- label: The text which will appear on the button.
- command: The terminal command ran by the user.
## Change user data
To run the terminal codes properly, you need to use the correct username and ros version in *userdata.json* (placed in *ReadFiles* folder).