

import "aos/dist/aos.css";
import {
  guarantees,
} from "./Data.tsx"; 

export default function Guarantees(){
    return(
        <section className="px-6 py-20 bg-gradient-to-br from-[#E4DDD2] to-[#D6CFC4]">
        <div className="max-w-6xl mx-auto">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-6">
              Our Commitment to You
            </h2>
            <p className="text-xl text-gray-600 max-w-2xl mx-auto">
              We stand behind our platform with guarantees that put your success
              first.
            </p>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
            {guarantees.map((guarantee, index) => (
              <div
                key={index}
                className="bg-white p-8 rounded-2xl shadow-lg text-center hover:shadow-xl transition-all duration-300"
              >
                <div className="mb-6">{guarantee.icon}</div>
                <h3 className="text-xl font-bold mb-4">{guarantee.title}</h3>
                <p className="text-gray-600 leading-relaxed">
                  {guarantee.description}
                </p>
              </div>
            ))}
          </div>
        </div>
      </section>
    )
}