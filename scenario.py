from link import Link
from intersection import Intersection
from car import Car


class Scenario:


    def load_scenario(scenario_nr):
        scenarios = [

            # ________________________________________________________________________________________________________
            #
            #                                       1D, 2 intersections, 2 cars
            # ________________________________________________________________________________________________________
            {
                'links': [Link(100, [-1, 0, -1, -1]), Link(50, [-1, 1, -1, 0]), Link(120, [-1, -1, -1, 1])],

                'intersections': [Intersection([-1, 1, -1, 0], [[0, 30], [30, 60], [0, 30], [30, 60]]),
                                Intersection([-1, 2, -1, 1], [[0, 30], [30, 60], [0, 30], [30, 60]])],

                'cars': [Car(0, 1, 6, 2), Car(2, 3, 12, 0)]
            },

            # ________________________________________________________________________________________________________
            #
            #                                       2D, 1 intersection, 2 cars
            # ________________________________________________________________________________________________________
            {
                'links': [Link(40, [0, -1, -1, -1]), Link(100, [-1, 0, -1, -1]), Link(50, [-1, -1, 0, -1]),
                        Link(120, [-1, -1, -1, 0])],
                'intersections': [Intersection([2, 3, 0, 1], [[0, 30], [30, 60], [0, 30], [30, 60]])],
                'cars': [Car(1, 1, 6, 3), Car(0, 0, 9, 2)]
            },



            # ________________________________________________________________________________________________________
            #
            #                                       2D, 6 intersections, 9 cars
            # ________________________________________________________________________________________________________
            {
                'links':  [Link(40, [0, -1, 3, -1]),
                         Link(100, [-1, 0, -1, -1]),
                         Link(50, [-1, -1, 0, -1]),
                         Link(120, [-1, 1, -1, 0]),
                         Link(40, [1, -1, 4, -1]),
                         Link(30, [-1, -1, 1, -1]),
                         Link(70, [-1, 2, -1, 1]),
                         Link(40, [2, -1, 5, -1]),
                         Link(60, [-1, -1, 2, -1]),
                         Link(90, [-1, -1, 2, -1]),
                         Link(80, [3, -1, -1, -1]),
                         Link(100, [-1, 3, -1, -1]),
                         Link(120, [-1, 4, -1, 3]),
                         Link(80, [4, -1, -1, -1]),
                         Link(70, [-1, 5, -1, 4]),
                         Link(80, [5, -1, -1, -1]),
                         Link(90, [-1, -1, -1, 5])],


                'intersections':  [Intersection([2, 3, 0, 1], [[0, 30], [30, 60], [0, 30], [30, 60]]),
                                 Intersection([5, 6, 4, 3], [[0, 30], [30, 60], [0, 30], [30, 60]]),
                                 Intersection([8, 9, 7, 6], [[0, 30], [30, 60], [0, 30], [30, 60]]),
                                 Intersection([0, 12, 10, 11], [[0, 30], [30, 60], [0, 30], [30, 60]]),
                                 Intersection([4, 14, 13, 12], [[0, 30], [30, 60], [0, 30], [30, 60]]),
                                 Intersection([7, 16, 15, 14], [[0, 30], [30, 60], [0, 30], [30, 60]])],

                'cars':[Car(1, 1, 8, 9),
                        Car(2, 2, 7, 10),
                        Car(5, 2, 4, 13),
                        Car(8, 2, 9, 15),
                        Car(16, 3, 11, 11),
                        Car(15, 0, 5, 8),
                        Car(13, 0, 14, 5),
                        Car(10, 0, 14, 2),
                        Car(11, 1, 2, 16)
                        ]
            }
        ]

        return scenarios[scenario_nr]