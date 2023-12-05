import 'package:flutter/material.dart';
import '../../constants.dart';

class CustomCard extends StatefulWidget {
  const CustomCard(
      {Key? key,
      required this.size,
      required this.icon,
      required this.title,
      required this.statusOn,
      required this.statusOff,
      required this.notifyOnState,
      required this.notifyOffState})
      : super(key: key);

  final Size size;
  final Icon icon;
  final String title;
  final String statusOn;
  final String statusOff;
  final Function notifyOnState;
  final Function notifyOffState;

  @override
  _CustomCardState createState() => _CustomCardState();
}

class _CustomCardState extends State<CustomCard>
    with SingleTickerProviderStateMixin {
  late AnimationController _animationController;
  late Animation<Alignment> _animation;
  bool isChecked = true;

  @override
  void initState() {
    _animationController = AnimationController(
      vsync: this,
      duration: Duration(milliseconds: 350),
    );

    _animation = Tween<Alignment>(
            begin: Alignment.bottomCenter, end: Alignment.topCenter)
        .animate(
      CurvedAnimation(
        parent: _animationController,
        curve: Curves.linear,
        reverseCurve: Curves.easeInBack,
      ),
    );
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      height: 120,
      width: widget.size.width * 0.4,
      decoration: BoxDecoration(
        color: kBgColor,
        borderRadius: BorderRadius.circular(20),
        boxShadow: [
          BoxShadow(
            color: Colors.black12,
            blurRadius: 8,
            offset: Offset(2, 2),
          ),
          BoxShadow(
            color: Colors.white,
            blurRadius: 0,
            offset: Offset(-2, -2),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.all(15),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(mainAxisAlignment: MainAxisAlignment.spaceBetween, children: [
              widget.icon,
              AnimatedBuilder(
                animation: _animationController,
                builder: (animation, child) {
                  return GestureDetector(
                    onTap: () {
                      setState(() {
                        if (widget.title == "AC ON") {
                          if (_animationController.isCompleted) {
                            _animationController.animateTo(20);
                          } else {
                            _animationController.animateTo(0);
                          }

                          isChecked = !isChecked;
                          if (!isChecked)
                            widget.notifyOnState();
                          else
                            widget.notifyOffState();
                        }
                      });
                    },
                    child: Container(
                      height: 30,
                      width: 20,
                      decoration: BoxDecoration(
                        borderRadius: BorderRadius.circular(20),
                        color: Colors.grey.shade50,
                        boxShadow: [
                          BoxShadow(
                            color: Colors.grey.shade200,
                            blurRadius: 8,
                            offset: Offset(2, 2),
                          ),
                          BoxShadow(
                            color: Colors.black12,
                            blurRadius: 5,
                            offset: Offset(-2, -2),
                          ),
                        ],
                      ),
                      child: Align(
                        alignment: _animation.value,
                        child: Container(
                          width: 15,
                          height: 15,
                          margin:
                              EdgeInsets.symmetric(vertical: 1, horizontal: 1),
                          decoration: BoxDecoration(
                            color:
                                isChecked ? Colors.grey.shade300 : kGreenColor,
                            shape: BoxShape.circle,
                          ),
                        ),
                      ),
                    ),
                  );
                },
              )
            ]),
            SizedBox(height: 10),
            Text(
              widget.title,
              style: TextStyle(
                fontSize: 14,
                fontWeight: FontWeight.bold,
                color: kBlueColor,
              ),
            ),
            Text(
              isChecked ? widget.statusOff : widget.statusOn,
              style: TextStyle(
                fontWeight: FontWeight.bold,
                color: isChecked ? Colors.grey.withOpacity(0.6) : kGreenColor,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
