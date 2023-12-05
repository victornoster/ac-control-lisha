import 'package:flutter/material.dart';
import 'package:LISHA_Control/LandingScreen/landing_screen.dart';
//import 'package:dcdg/dcdg.dart';

void main() {
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Lisha App',
      theme: ThemeData(
          visualDensity: VisualDensity.adaptivePlatformDensity,
          fontFamily: 'Poppins'),
      home: LandingScreen(),
    );
  }
}
