/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolferNames.h"

namespace OpenRCT2::Golf
{
    const std::vector<std::string>& defaultFirstNames()
    {
    static const std::vector<std::string> names = {
            "James", "Mary", "Robert", "Patricia", "John", "Jennifer", "Michael", "Linda", "William", "Elizabeth", "David",
            "Barbara", "Richard", "Susan", "Joseph", "Jessica", "Thomas", "Sarah", "Charles", "Karen", "Christopher",
            "Lisa", "Daniel", "Nancy", "Matthew", "Betty", "Anthony", "Sandra", "Mark", "Margaret", "Donald", "Ashley",
            "Steven", "Kimberly", "Paul", "Emily", "Andrew", "Donna", "Joshua", "Michelle", "Kenneth", "Carol", "Kevin",
            "Amanda", "Brian", "Melissa", "George", "Deborah", "Timothy", "Stephanie", "Ronald", "Rebecca", "Edward",
            "Laura", "Jason", "Sharon", "Jeffrey", "Cynthia", "Ryan", "Kathleen", "Jacob", "Amy", "Gary", "Angela",
            "Nicholas", "Shirley", "Eric", "Anna", "Jonathan", "Brenda", "Stephen", "Pamela", "Larry", "Emma", "Justin",
            "Nicole", "Scott", "Helen", "Brandon", "Samantha", "Benjamin", "Katherine", "Samuel", "Christine", "Gregory",
            "Debra", "Alexander", "Rachel", "Frank", "Carolyn", "Patrick", "Janet", "Raymond", "Maria", "Jack", "Catherine",
            "Dennis", "Heather", "Jerry", "Diane", "Tyler", "Olivia", "Aaron", "Julie", "Jose", "Joyce", "Adam", "Victoria",
            "Nathan", "Ruth", "Henry", "Virginia", "Douglas", "Lauren", "Zachary", "Kelly", "Peter", "Christina", "Kyle",
            "Joan", "Ethan", "Evelyn", "Walter", "Judith", "Noah", "Andrea", "Jeremy", "Hannah", "Christian", "Megan",
            "Keith", "Cheryl", "Roger", "Jacqueline", "Terry", "Martha", "Austin", "Gloria", "Sean", "Teresa", "Gerald",
            "Ann", "Carl", "Sara", "Harold", "Madison", "Dylan", "Frances", "Arthur", "Kathryn", "Lawrence", "Janice",
            "Jordan", "Jean", "Jesse", "Abigail", "Bryan", "Alice", "Billy", "Julia", "Bruce", "Judy", "Gabriel", "Sophia",
            "Joe", "Grace", "Logan", "Denise", "Alan", "Amber", "Juan", "Doris", "Albert", "Marilyn", "Willie", "Danielle",
            "Elijah", "Beverly", "Wayne", "Isabella", "Randy", "Theresa", "Vincent", "Diana", "Mason", "Natalie", "Roy",
            "Brittany", "Ralph", "Charlotte", "Bobby", "Marie", "Russell", "Kayla", "Bradley", "Alexis", "Philip", "Lori",
            "Eugene", "Hugo", "Liam", "Oscar", "Felix", "Ingrid", "Astrid", "Sven", "Lars", "Freya", "Mateo", "Lucia",
            "Chloe", "Leo", "Ivy", "Hazel", "Silas", "Rowan", "Wesley", "Marcus", "Elena", "Tobias", "Clara", "Nora",
            "Declan", "Fiona", "Gavin", "Maeve", "Callum", "Isla", "Finn", "Ronan", "Sienna", "Jasper", "Willa", "Otis",
            "Maude", "Percy", "Edith", "Cedric", "Beatrice", "Rufus", "Winifred",
    };
        return names;
    }

    const std::vector<std::string>& defaultFamilyNames()
    {
    static const std::vector<std::string> names = {
            "Smith", "Johnson", "Williams", "Brown", "Jones", "Garcia", "Miller", "Davis", "Rodriguez", "Martinez",
            "Hernandez", "Lopez", "Gonzalez", "Wilson", "Anderson", "Thomas", "Taylor", "Moore", "Jackson", "Martin", "Lee",
            "Perez", "Thompson", "White", "Harris", "Sanchez", "Clark", "Ramirez", "Lewis", "Robinson", "Walker", "Young",
            "Allen", "King", "Wright", "Scott", "Torres", "Nguyen", "Hill", "Flores", "Green", "Adams", "Nelson", "Baker",
            "Hall", "Rivera", "Campbell", "Mitchell", "Carter", "Roberts", "Gomez", "Phillips", "Evans", "Turner", "Diaz",
            "Parker", "Cruz", "Edwards", "Collins", "Reyes", "Stewart", "Morris", "Morales", "Murphy", "Cook", "Rogers",
            "Gutierrez", "Ortiz", "Morgan", "Cooper", "Peterson", "Bailey", "Reed", "Kelly", "Howard", "Ramos", "Kim",
            "Cox", "Ward", "Richardson", "Watson", "Brooks", "Chavez", "Wood", "James", "Bennett", "Gray", "Mendoza",
            "Ruiz", "Hughes", "Price", "Alvarez", "Castillo", "Sanders", "Patel", "Myers", "Long", "Ross", "Foster",
            "Jimenez", "Powell", "Jenkins", "Perry", "Russell", "Sullivan", "Bell", "Coleman", "Butler", "Henderson",
            "Barnes", "Gonzales", "Fisher", "Vasquez", "Simmons", "Romero", "Jordan", "Patterson", "Alexander", "Hamilton",
            "Graham", "Reynolds", "Griffin", "Wallace", "Moreno", "West", "Cole", "Hayes", "Bryant", "Herrera", "Gibson",
            "Ellis", "Tran", "Medina", "Aguilar", "Stevens", "Murray", "Ford", "Castro", "Marshall", "Owens", "Harrison",
            "Fernandez", "McDonald", "Woods", "Washington", "Kennedy", "Wells", "Vargas", "Henry", "Chen", "Freeman",
            "Webb", "Tucker", "Guzman", "Burns", "Crawford", "Olson", "Simpson", "Porter", "Hunter", "Gordon", "Mendez",
            "Silva", "Shaw", "Snyder", "Mason", "Dixon", "Munoz", "Hunt", "Hicks", "Holmes", "Palmer", "Wagner", "Black",
            "Robertson", "Boyd", "Rose", "Stone", "Salazar", "Fox", "Warren", "Mills", "Meyer", "Rice", "Schmidt", "Garza",
            "Daniels", "Ferguson", "Nichols", "Stephens", "Soto", "Weaver", "Ryan", "Gardner", "Payne", "Grant", "Dunn",
            "Kelley", "Spencer", "Hawkins", "Arnold", "Pierce", "Vazquez", "Hansen", "Peters", "Santos", "Hart", "Bradley",
            "Knight", "Elliott", "Cunningham", "Duncan", "Armstrong", "Hudson", "Carroll", "Lane", "Riley", "Andrews",
            "Alvarado", "Ray", "Delgado", "Berry", "Perkins", "Hoffman", "Johnston", "Matthews", "Pena", "Richards",
            "Contreras", "Willis", "Carpenter", "Lawrence", "Sandoval", "Guerrero", "George", "Chapman", "Rios", "Estrada",
            "Ortega", "Watkins", "Greene", "Nunez", "Wheeler", "Valdez", "Harper", "Burke", "Larson", "Santiago",
            "Maldonado", "Morrison", "Franklin", "Carlson", "Austin", "Dominguez", "Carr", "Lawson", "Jacobs", "Obrien",
            "Lynch", "Singh", "Vega", "Bishop", "Montgomery", "Oliver", "Jensen", "Harvey", "Williamson", "Gilbert", "Dean",
            "Sims", "Espinoza", "Howell", "Li", "Wong", "Reid", "Hanson", "Le", "McCoy", "Garrett", "Burton", "Fuller",
            "Wang", "Weber", "Welch", "Rojas", "Lucas", "Marquez", "Fields", "Yang", "Little", "Banks", "Padilla",
            "Day", "Walsh", "Bowman", "Schultz", "Luna", "Fowler", "Mejia", "Davidson", "Acosta", "Brewer", "May",
            "Holland", "Juarez", "Newman", "Pearson", "Curtis", "Cortez", "Douglas", "Schneider", "Joseph", "Barrett",
            "Navarro", "Figueroa", "Keller", "Avila", "Wade", "Molina", "Stanley", "Hopkins", "Campos", "Barnett", "Bates",
            "Chambers", "Caldwell", "Beck", "Lambert", "Miranda", "Byrd", "Craig", "Ayala", "Lowe", "Frazier", "Powers",
            "Neal", "Leonard", "Gregory", "Carrillo", "Sutton", "Fleming", "Rhodes", "Shelton", "Schwartz", "Norris",
            "Jennings", "Watts", "Duran", "Walters", "Cohen", "McDaniel", "Moran", "Steele", "McClain", "Thornton",
            "Gill",
    };
        return names;
    }
} // namespace OpenRCT2::Golf
