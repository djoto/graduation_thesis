   M S B A M L   `   `   `  ����  DepthBasics-WPF5    /Microsoft.Samples.Kinect.DepthBasics.MainWindow   5   	   R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   -   5          � Title$  Depth Basics��5   	   $	��600��5   	   $	��700��6    � 	Resources 5         MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush �� 
TargetType �� Property �� Value  ScreenshotButton �� 
TargetType �� Orientation ��Name	 �� TextAlignment
 ��Name �� Triggers P� Property P� Value �� 
TargetName��%a  &	        &	 #     &	 F     '�� i     &	 �     7  7  �� 5   
   $ 	#ff6e6e6e��62   �� 5	   
   $ 	#ff52318f��64   �� 5
   
   $ 	#ff00BCF2��62   �� 5   
   " ��6   .�5      ��  �@J�SnapsToDevicePixels . 64   5      �� 5   
   " ��6   .�5      ��  � �� 5      �� 5      " ��6&   .R�5      � .+�5      �� $ 
Horizontal=�6)   
������ .+�5   "   �� $ ScreenshotText��6,   #��z��6D   $
	 Left=�6e   $��Center=�6{   8��  $��15(�6�   � 5   "   $��9,0,0,0q�6'   .+�5   &   �� $
 ScreenshotNormal��6,   $!��Images\ScreenshotNormal.png��6F   $
��None=�6k   $��Center=�6z   �� 5   &   $
 ScreenshotHover��6,   $ ��Images\ScreenshotHover.png��6E   $
��None=�6i   $��Center=�6x   $y�	Collapsed=�6�   5   #   5      5       5      P� 5       �@J�IsMouseOver . . �5   "   �� $ ScreenshotNormal��6Q    � y�$ 	Collapsed=�6?   6Q   �� 5   "   $ ScreenshotHover��6O    � y�$ Visible=�6?   6O   �� 5   "   8  $ ScreenshotText��6h    � ��5       5"      5$      5%      5&      .��5'      � $��	10 0 10 0q�6    � RowDefinitions 5(   
   �� 5)      $
��Auto �6   �� 5*      $��* �6   �� 5+      $
��Auto �6   �� 5,      $
��Auto �6   .+�5/   
   �� $��0��6   $��Images\Logo.png��6   $
��Left=�66   $
��Fill=�6Q   $��32��6`   $��81��6l   $��0 10 0 5q�6w   �� 50   
   $��0��6   $��0 0 -1 0q�6!   $��Right=�63   $��Bottom=�6O   #��[  6j   $��Segoe UI)�6�   $��18(�6�   Depth Basics6�   �� 51   
   $��0��6   $��Images\Status.png��6   $
��None=�68   $��Center=�6G   $��0 0 0 5q�6d   (� 53   
   $��1��6   $��Center=�6   .��54      �� $��UniformToFill=�63   ��6   �� *ImageSource+63   55      �� 57   
   -   5        $��2��57      #��[ 6   $��
Screenshot��6H   $
��Auto��6]   $��Right=�6k   $��Center=�6�   $��
10 10 0 10q�6�   6�   �� 59   
   -   5        $��	statusBar��59   ?   $��3��6   $��Stretch=�6!   $��Bottom=�6P   
�������#��[  6~   .@�5:      �� ��6   �� *
StatusText+5;      �� 5<   
   -   5         ��Name$ MaxDepthInput��5<      $
��Left=�6)   $��22��6D   $��
250,15,0,0q�6P   $��2��6d    �� TextWrapping$
 Wrap=�6q   $	��Top=�6�   $��65��6�   $
��1280��6�    �� TextAlignment$ Center=�6�   $��14(�6�    J� RenderTransformOrigin$ 
0.507,0.87&�6�   �� 5=   
   -   5         ��Name$ ResetButton��5=      $��Resetuj��6&   $
��Left=�68   $��
463,18,0,0q�6S   $��2��6g   $	��Top=�6t   $��56��6�   $ 0.543,0.333&�6�    5>      ��%^   '��       �� 5?      " ��6   .�5@      ��  �@��CornerRadius$ 5��65   5A      5B      5C      �� 5D   
   -   5         ��Name$ labelInputDepth��5D      $%��Unos maksimalne dubine piksela:��6)   $
��Left=�6S   $��	30,11,0,0q�6n   $��2��6�   $	��Top=�6�   $	��215��6�   $��14(�6�   �� 5E   
   -   5        $ classifyButton��5E      $��
KLASIFIKUJ��6)   $
��Left=�6>   $��
348,11,0,0q�6Y   $��2��6m   $	��Top=�6z   $��86��6�   $��31��6�   ���	#FFDDDDDD$��14(�6�   $��SemiBold"�6�   $��Italic$�6�    5F      ��%^   '��       �� 5G      " ��6   .�5H      ��  �@��CornerRadius$ 5��65   5I      5J      5K      5M      5N      